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

/*
 * Build an exact full-state BFS distance table on the host.
 * This is only used as the H1 oracle and is NOT part of the RV32I target.
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
        fprintf(stderr, "PDB verification failed before H1.\n");
        return 1;
    }

    printf("Building exact BFS oracle...\n");
    double bfs_start = now_seconds();

    uint8_t *exact = build_exact_distance_table();
    if (!exact)
        return 1;

    double bfs_end = now_seconds();
    printf("Exact BFS build time: %.3f seconds\n", bfs_end - bfs_start);

    printf("Running FULL H1 admissibility check over all %u states...\n",
           (unsigned)STATES);

    double h1_start = now_seconds();

    uint32_t checked = 0;
    uint32_t violations = 0;
    uint8_t max_h = 0;
    uint8_t max_exact = 0;
    uint8_t largest_gap = 0;

    for (uint16_t p = 0; p < PERMUTATIONS; ++p) {
        for (uint16_t o = 0; o < ORIENTATIONS; ++o) {
            uint32_t rank = (uint32_t)p * ORIENTATIONS + o;
            uint8_t h = heuristic(p, o);
            uint8_t d = exact[rank];

            if (h > max_h)
                max_h = h;

            if (d > max_exact)
                max_exact = d;

            if ((uint8_t)(d - h) > largest_gap)
                largest_gap = (uint8_t)(d - h);

            if (h > d) {
                ++violations;

                fprintf(stderr,
                        "\nH1 FAIL: inadmissible heuristic at "
                        "p=%u o=%u rank=%u: h=%u exact=%u\n",
                        (unsigned)p,
                        (unsigned)o,
                        (unsigned)rank,
                        (unsigned)h,
                        (unsigned)d);

                free(exact);
                return 1;
            }

            ++checked;
        }

        if ((p % 250U) == 0U || p + 1U == PERMUTATIONS) {
            double percent = 100.0 * (double)checked / (double)STATES;
            printf("\rChecked %u / %u states (%.2f%%)",
                   (unsigned)checked,
                   (unsigned)STATES,
                   percent);
            fflush(stdout);
        }
    }

    double h1_end = now_seconds();
    putchar('\n');

    printf("H1 PASS: heuristic was admissible for all %u states.\n",
           (unsigned)checked);
    printf("Violations: %u\n", (unsigned)violations);
    printf("Maximum heuristic value: %u\n", (unsigned)max_h);
    printf("Exact state-space diameter: %u\n", (unsigned)max_exact);
    printf("Largest exact-distance minus heuristic gap: %u\n",
           (unsigned)largest_gap);
    printf("H1 wall-clock time: %.3f seconds\n", h1_end - h1_start);

    free(exact);
    return 0;
}
