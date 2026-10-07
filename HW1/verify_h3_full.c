#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#define NO_RENDER
#define main stage3_program_main
#include "solver.c"
#undef main

enum { STATES = PERMUTATIONS * ORIENTATIONS };

static double now_seconds(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

/* Reconstruct the full state_t that corresponds to coordinate pair (p, o). */
static void state_from_coordinates(uint16_t p, uint16_t o, state_t *state)
{
    state_t orientation_state;

    unrank_permutation(p, state);
    unrank_orientation(o, &orientation_state);

    for (uint8_t i = 0; i < CUBIES; ++i)
        state->o[i] = orientation_state.o[i];
}

/*
 * Build an exact full-state BFS distance table on the host.
 * This table is ONLY the host oracle for H3; it is not part of the target solver.
 */
static uint8_t *build_exact_distance_table(void)
{
    uint8_t *distance = malloc((size_t)STATES * sizeof *distance);
    uint32_t *queue = malloc((size_t)STATES * sizeof *queue);

    if (!distance || !queue) {
        free(distance);
        free(queue);
        return NULL;
    }

    memset(distance, UINT8_MAX, (size_t)STATES);

    uint32_t head = 0;
    uint32_t tail = 1;
    uint8_t diameter = 0;

    distance[0] = 0;
    queue[0] = 0;

    while (head < tail) {
        uint32_t here = queue[head++];
        uint16_t p = (uint16_t)(here / ORIENTATIONS);
        uint16_t o = (uint16_t)(here % ORIENTATIONS);
        uint8_t next_distance = (uint8_t)(distance[here] + 1U);

        for (uint8_t face = 0; face < 3U; ++face) {
            uint16_t next_p = p;
            uint16_t next_o = o;

            for (uint8_t turn = 0; turn < 3U; ++turn) {
                next_p = permutation_turn(next_p, face);
                next_o = orientation_turn(next_o, face);

                uint32_t there =
                    (uint32_t)next_p * ORIENTATIONS + next_o;

                if (distance[there] == UINT8_MAX) {
                    distance[there] = next_distance;
                    queue[tail++] = there;

                    if (next_distance > diameter)
                        diameter = next_distance;
                }
            }
        }
    }

    free(queue);

    if (tail != STATES || diameter != MAX_DEPTH) {
        fprintf(stderr,
                "Exact BFS failed: reached %u/%u states, diameter=%u\n",
                (unsigned)tail,
                (unsigned)STATES,
                (unsigned)diameter);
        free(distance);
        return NULL;
    }

    printf("Exact BFS oracle ready: %u states, diameter %u\n",
           (unsigned)tail,
           (unsigned)diameter);

    return distance;
}

int main(void)
{
    printf("Building Stage 3 transition tables and PDBs...\n");

    build_transition_tables();

    if (!build_permutation_pdb() || !build_orientation_pdb()) {
        fprintf(stderr, "Could not build Stage 3 PDBs.\n");
        return 1;
    }

    if (!verify_pdbs()) {
        fprintf(stderr, "PDB verification failed before H3.\n");
        return 1;
    }

    printf("Building exact BFS oracle...\n");
    double bfs_start = now_seconds();

    uint8_t *exact = build_exact_distance_table();
    if (!exact)
        return 1;

    double bfs_end = now_seconds();
    printf("Exact BFS build time: %.3f seconds\n", bfs_end - bfs_start);

    printf("Running FULL H3 over all %u states...\n", (unsigned)STATES);
    double h3_start = now_seconds();

    uint32_t checked = 0;

    for (uint16_t p = 0; p < PERMUTATIONS; ++p) {
        for (uint16_t o = 0; o < ORIENTATIONS; ++o) {
            uint32_t rank = (uint32_t)p * ORIENTATIONS + o;
            state_t state;
            uint8_t path[MAX_DEPTH];
            uint8_t length = 0;

            state_from_coordinates(p, o, &state);

            if (!ida_solve(state, path, &length)) {
                fprintf(stderr,
                        "\nH3 FAIL: solver returned no solution at "
                        "p=%u o=%u rank=%u exact=%u\n",
                        (unsigned)p,
                        (unsigned)o,
                        (unsigned)rank,
                        (unsigned)exact[rank]);
                free(exact);
                return 1;
            }

            if (length != exact[rank]) {
                fprintf(stderr,
                        "\nH3 FAIL: non-optimal length at "
                        "p=%u o=%u rank=%u: solver=%u exact=%u\n",
                        (unsigned)p,
                        (unsigned)o,
                        (unsigned)rank,
                        (unsigned)length,
                        (unsigned)exact[rank]);
                free(exact);
                return 1;
            }

            if (!validate_solution(state, path, length)) {
                fprintf(stderr,
                        "\nH3 FAIL: returned path does not solve state at "
                        "p=%u o=%u rank=%u\n",
                        (unsigned)p,
                        (unsigned)o,
                        (unsigned)rank);
                free(exact);
                return 1;
            }

            ++checked;
        }

        if ((p % 100U) == 0U || p + 1U == PERMUTATIONS) {
            double percent = 100.0 * (double)checked / (double)STATES;
            printf("\rChecked %u / %u states (%.2f%%)",
                   (unsigned)checked,
                   (unsigned)STATES,
                   percent);
            fflush(stdout);
        }
    }

    double h3_end = now_seconds();
    putchar('\n');

    printf("H3 PASS: all %u states matched the exact BFS distance.\n",
           (unsigned)checked);
    printf("H3 wall-clock time: %.3f seconds\n", h3_end - h3_start);

    free(exact);
    return 0;
}
