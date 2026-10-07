#include <stdint.h>
#include <stdio.h>

#define NO_RENDER
#define main stage3_program_main
#include "solver.c"
#undef main

static int verify_transition_tables(void)
{
    uint16_t perm_max = 0;
    uint16_t ori_max = 0;
    uint32_t perm_checked = 0;
    uint32_t ori_checked = 0;
    state_t state;

    /* Verify every permutation transition entry against direct recomputation. */
    for (uint16_t rank = 0; rank < PERMUTATIONS; ++rank) {
        unrank_permutation(rank, &state);

        for (uint8_t face = 0; face < 3U; ++face) {
            state_t next = quarter_turn(state, face);
            uint16_t expected = rank_permutation(&next);
            uint16_t actual = permutation_next[face][rank];

            if (actual != expected) {
                fprintf(stderr,
                        "H2 FAIL: permutation_next[%u][%u] = %u, expected %u\n",
                        (unsigned)face,
                        (unsigned)rank,
                        (unsigned)actual,
                        (unsigned)expected);
                return 0;
            }

            if (actual >= PERMUTATIONS) {
                fprintf(stderr,
                        "H2 FAIL: permutation transition out of range: "
                        "face=%u rank=%u value=%u\n",
                        (unsigned)face,
                        (unsigned)rank,
                        (unsigned)actual);
                return 0;
            }

            if (actual > perm_max)
                perm_max = actual;

            ++perm_checked;
        }
    }

    /* Verify every orientation transition entry against direct recomputation. */
    for (uint16_t rank = 0; rank < ORIENTATIONS; ++rank) {
        unrank_orientation(rank, &state);

        for (uint8_t face = 0; face < 3U; ++face) {
            state_t next = quarter_turn(state, face);
            uint16_t expected = rank_orientation(&next);
            uint16_t actual = orientation_next[face][rank];

            if (actual != expected) {
                fprintf(stderr,
                        "H2 FAIL: orientation_next[%u][%u] = %u, expected %u\n",
                        (unsigned)face,
                        (unsigned)rank,
                        (unsigned)actual,
                        (unsigned)expected);
                return 0;
            }

            if (actual >= ORIENTATIONS) {
                fprintf(stderr,
                        "H2 FAIL: orientation transition out of range: "
                        "face=%u rank=%u value=%u\n",
                        (unsigned)face,
                        (unsigned)rank,
                        (unsigned)actual);
                return 0;
            }

            if (actual > ori_max)
                ori_max = actual;

            ++ori_checked;
        }
    }

    if (perm_max != PERMUTATIONS - 1U) {
        fprintf(stderr,
                "H2 FAIL: permutation transition max = %u, expected %u\n",
                (unsigned)perm_max,
                (unsigned)(PERMUTATIONS - 1U));
        return 0;
    }

    if (ori_max != ORIENTATIONS - 1U) {
        fprintf(stderr,
                "H2 FAIL: orientation transition max = %u, expected %u\n",
                (unsigned)ori_max,
                (unsigned)(ORIENTATIONS - 1U));
        return 0;
    }

    printf("Permutation transitions checked: %u entries\n",
           (unsigned)perm_checked);
    printf("Orientation transitions checked: %u entries\n",
           (unsigned)ori_checked);
    printf("Permutation transition max: %u\n", (unsigned)perm_max);
    printf("Orientation transition max: %u\n", (unsigned)ori_max);

    printf("Solved-coordinate transition entries:\n");
    for (uint8_t face = 0; face < 3U; ++face) {
        printf("  face %u: permutation_next[%u][0] = %u, "
               "orientation_next[%u][0] = %u\n",
               (unsigned)face,
               (unsigned)face,
               (unsigned)permutation_next[face][0],
               (unsigned)face,
               (unsigned)orientation_next[face][0]);
    }

    return 1;
}

static int verify_pdb_tables_verbose(void)
{
    uint8_t perm_max = 0;
    uint8_t ori_max = 0;
    uint32_t perm_count = 0;
    uint32_t ori_count = 0;

    if (perm_dist[0] != 0U) {
        fprintf(stderr,
                "H2 FAIL: perm_dist[0] = %u, expected 0\n",
                (unsigned)perm_dist[0]);
        return 0;
    }

    if (ori_dist[0] != 0U) {
        fprintf(stderr,
                "H2 FAIL: ori_dist[0] = %u, expected 0\n",
                (unsigned)ori_dist[0]);
        return 0;
    }

    for (uint16_t i = 0; i < PERMUTATIONS; ++i) {
        if (perm_dist[i] == UINT8_MAX) {
            fprintf(stderr,
                    "H2 FAIL: perm_dist[%u] was not populated\n",
                    (unsigned)i);
            return 0;
        }

        if (perm_dist[i] > perm_max)
            perm_max = perm_dist[i];

        ++perm_count;
    }

    for (uint16_t i = 0; i < ORIENTATIONS; ++i) {
        if (ori_dist[i] == UINT8_MAX) {
            fprintf(stderr,
                    "H2 FAIL: ori_dist[%u] was not populated\n",
                    (unsigned)i);
            return 0;
        }

        if (ori_dist[i] > ori_max)
            ori_max = ori_dist[i];

        ++ori_count;
    }

    if (perm_max != 7U) {
        fprintf(stderr,
                "H2 FAIL: permutation PDB max = %u, expected 7\n",
                (unsigned)perm_max);
        return 0;
    }

    if (ori_max != 6U) {
        fprintf(stderr,
                "H2 FAIL: orientation PDB max = %u, expected 6\n",
                (unsigned)ori_max);
        return 0;
    }

    printf("Permutation PDB populated entries: %u / %u\n",
           (unsigned)perm_count,
           (unsigned)PERMUTATIONS);
    printf("Orientation PDB populated entries: %u / %u\n",
           (unsigned)ori_count,
           (unsigned)ORIENTATIONS);
    printf("Permutation PDB solved entry: %u\n",
           (unsigned)perm_dist[0]);
    printf("Orientation PDB solved entry: %u\n",
           (unsigned)ori_dist[0]);
    printf("Permutation PDB max: %u\n", (unsigned)perm_max);
    printf("Orientation PDB max: %u\n", (unsigned)ori_max);

    return 1;
}



static void write_half_table(FILE *fp, const char *label,
                             const uint16_t *data, uint32_t count)
{
    fprintf(fp, "%s:\n", label);
    for (uint32_t i = 0; i < count; ++i) {
        if (i % 16U == 0U)
            fputs("    .half ", fp);
        else
            fputc(',', fp);

        fprintf(fp, "%u", (unsigned)data[i]);

        if (i % 16U == 15U || i + 1U == count)
            fputc('\n', fp);
    }
}

static void write_byte_table(FILE *fp, const char *label,
                             const uint8_t *data, uint32_t count)
{
    fprintf(fp, "%s:\n", label);
    for (uint32_t i = 0; i < count; ++i) {
        if (i % 32U == 0U)
            fputs("    .byte ", fp);
        else
            fputc(',', fp);

        fprintf(fp, "%u", (unsigned)data[i]);

        if (i % 32U == 31U || i + 1U == count)
            fputc('\n', fp);
    }
}

static int write_reference_table(const char *filename)
{
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        perror(filename);
        return 0;
    }

    fputs("# Generated from solver_stage3_stage2style.c after H2 verification\n", fp);
    fputs(".align 2\n", fp);

    write_half_table(fp, "perm_0", permutation_next[0], PERMUTATIONS);
    write_half_table(fp, "ori_0", orientation_next[0], ORIENTATIONS);
    write_half_table(fp, "perm_1", permutation_next[1], PERMUTATIONS);
    write_half_table(fp, "ori_1", orientation_next[1], ORIENTATIONS);
    write_half_table(fp, "perm_2", permutation_next[2], PERMUTATIONS);
    write_half_table(fp, "ori_2", orientation_next[2], ORIENTATIONS);
    write_byte_table(fp, "dist_p", perm_dist, PERMUTATIONS);
    write_byte_table(fp, "dist_o", ori_dist, ORIENTATIONS);

    if (fclose(fp) != 0) {
        perror(filename);
        return 0;
    }

    return 1;
}


int main(void)
{
    printf("Building transition tables...\n");
    build_transition_tables();

    printf("Building PDBs...\n");

    if (!build_permutation_pdb()) {
        fprintf(stderr,
                "H2 FAIL: permutation PDB did not reach all %u states\n",
                (unsigned)PERMUTATIONS);
        return 1;
    }

    if (!build_orientation_pdb()) {
        fprintf(stderr,
                "H2 FAIL: orientation PDB did not reach all %u states\n",
                (unsigned)ORIENTATIONS);
        return 1;
    }

    printf("Checking transition tables...\n");
    if (!verify_transition_tables())
        return 1;

    printf("Checking PDB tables...\n");
    if (!verify_pdb_tables_verbose())
        return 1;

    printf("H2 PASS: every required table was fully verified.\n");

    if (!write_reference_table("table_generated.s"))
        return 1;

    printf("Generated table_generated.s\n");
    return 0;
}
