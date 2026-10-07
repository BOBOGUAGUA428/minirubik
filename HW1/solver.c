#include <stdint.h>
#include <stdio.h>
#include <string.h>
enum {
    CUBIES = 7,
    PERMUTATIONS = 5040,
    ORIENTATIONS = 729,
    MOVES = 9,
    MAX_DEPTH = 11,
    NO_FACE = 3
};

typedef struct {
    uint8_t p[CUBIES], o[CUBIES];
} state_t;

#ifndef NO_RENDER
static const char *const move_names[MOVES] = {
    "R", "R2", "R'", "B", "B2", "B'", "D", "D2", "D'"
};
#endif

static const uint8_t inverse_move[MOVES] = {2, 1, 0, 5, 4, 3, 8, 7, 6};
static const uint8_t move_face[MOVES] = {0, 0, 0, 1, 1, 1, 2, 2, 2};
static const uint8_t move_turns[MOVES] = {1, 2, 3, 1, 2, 3, 1, 2, 3};
static const uint8_t source[3][CUBIES] = {
    {1, 4, 2, 0, 3, 5, 6},
    {0, 1, 2, 4, 5, 6, 3},
    {0, 2, 5, 3, 1, 4, 6},
};

static const uint8_t twist[3][CUBIES] = {
    {1, 2, 0, 2, 1, 0, 0},
    {0, 0, 0, 1, 2, 1, 2},
    {0, 0, 0, 0, 0, 0, 0},
};

/* Stage 2 tables are still built at run time. */
static uint16_t permutation_next[3][PERMUTATIONS];
static uint16_t orientation_next[3][ORIENTATIONS];
static uint8_t perm_dist[PERMUTATIONS];
static uint8_t ori_dist[ORIENTATIONS];

/* One fixed queue is reused by both PDB BFS builds. No heap is needed. */
static uint16_t pdb_queue[PERMUTATIONS];

/* Orientation addition modulo 3. Inputs are always 0..2, so one subtract is enough. */
static uint8_t add_mod3(uint8_t a, uint8_t b)
{
    uint8_t value = (uint8_t)(a + b);
    if (value >= 3U)
        value = (uint8_t)(value - 3U);
    return value;
}

/* Same cube move definition as the teacher's baseline, without % 3. */
static state_t quarter_turn(state_t state, uint8_t face)
{
    state_t result;
    for (uint8_t i = 0; i < CUBIES; ++i) {
        uint8_t from = source[face][i]; //source告訴我們「轉完之後，第 i 格是從原本哪一格的corner」
        result.p[i] = state.p[from];
        result.o[i] = add_mod3(state.o[from], twist[face][i]);
        //就是把方塊原本的朝向，加上旋轉造成的偏移量 再去mod 3，得到新的朝向
    }
    return result;
}

/* Same nine moves as Stage 2, but face/turn are looked up instead of / and %. */
static state_t apply_move(state_t state, uint8_t move)
{
    //直接做兩張非常小的表 line 25 26
    uint8_t face = move_face[move];
    uint8_t turns = move_turns[move];
    for (uint8_t i = 0; i < turns; ++i)//用loop跑做幾次轉
        state = quarter_turn(state, face);
    return state;
}

/* Fixed small multipliers used by Lehmer ranking, written with shifts/adds. */
//RV32I base ISA 沒有 multiply，所以 fixed-constant multiplication 很適合用 shift/add 實作

static uint16_t multiply_small(uint16_t x, uint8_t factor)
{
    switch (factor) { //x<<n 為 x*2^n
    case 7: return (uint16_t)((x << 3) - x);
    case 6: return (uint16_t)((x << 2) + (x << 1));
    case 5: return (uint16_t)((x << 2) + x);
    case 4: return (uint16_t)(x << 2);
    case 3: return (uint16_t)((x << 1) + x);
    case 2: return (uint16_t)(x << 1);
    default: return x;
    }
}

/* Same Lehmer/factoradic permutation coordinate as Stage 2. */
static uint16_t rank_permutation(const state_t *state)
{
    uint16_t rank = 0;
    for (uint8_t i = 0; i < CUBIES; ++i) {
        uint8_t smaller = 0;
        //i+1u是因為要算有幾個比自己小的數字，從i+1開始算到CUBIES-1
        for (uint8_t j = (uint8_t)(i + 1U); j < CUBIES; ++j) {
            if (state->p[j] < state->p[i])
                ++smaller;
        }
        rank = (uint16_t)(multiply_small(rank, (uint8_t)(CUBIES - i)) + smaller);
    }
    return rank;
}

/* Same base-3 orientation coordinate as Stage 2, without multiplication. */
static uint16_t rank_orientation(const state_t *state)
{
    uint16_t rank = 0;
    //rank*3=2 rank + rank
    for (uint8_t i = 0; i < 6U; ++i)
        rank = (uint16_t)((rank << 1) + rank + state->o[i]);
    return rank;
}

/*
 * Decode one permutation rank without division.
 * Each quotient digit is only 0..6, so repeated subtraction is bounded and small.
 */
static void unrank_permutation(uint16_t rank, state_t *state)
{
    static const uint16_t factorial[CUBIES] = {720, 120, 24, 6, 2, 1, 1};
    uint8_t available[CUBIES] = {0, 1, 2, 3, 4, 5, 6};
    for (uint8_t i = 0; i < CUBIES; ++i) {
        uint16_t f = factorial[i];
        uint8_t q = 0;
        while (rank >= f) {
            rank = (uint16_t)(rank - f);
            ++q;
        }
        state->p[i] = available[q];
        for (uint8_t j = q; (uint8_t)(j + 1U) < (uint8_t)(CUBIES - i); ++j)
            available[j] = available[j + 1U];
    }
    for (uint8_t i = 0; i < CUBIES; ++i)
        state->o[i] = 0;
}

/* Decode one orientation rank in base 3 without /3 or %3. */
static void unrank_orientation(uint16_t rank, state_t *state)
{
    static const uint16_t power3[6] = {243, 81, 27, 9, 3, 1};
    uint8_t sum_mod3 = 0;
    for (uint8_t i = 0; i < CUBIES; ++i)
        state->p[i] = i;
    for (uint8_t i = 0; i < 6U; ++i) {
        uint16_t p = power3[i];
        uint8_t value = 0;
        if (rank >= (uint16_t)(p << 1)) {
            rank = (uint16_t)(rank - (p << 1));
            value = 2;
        } else if (rank >= p) {
            rank = (uint16_t)(rank - p);
            value = 1;
        }
        state->o[i] = value;
        sum_mod3 = add_mod3(sum_mod3, value);
    }
    state->o[6] = sum_mod3 == 0U ? 0U : (uint8_t)(3U - sum_mod3);
}

static int valid(const state_t *state)
{
    uint8_t sum_mod3 = 0;
    for (uint8_t i = 0; i < CUBIES; ++i) {
        if (state->p[i] >= CUBIES || state->o[i] >= 3U)
            return 0;
        for (uint8_t j = 0; j < i; ++j) {
            if (state->p[j] == state->p[i])
                return 0;
        }
        sum_mod3 = add_mod3(sum_mod3, state->o[i]);
    }
    return sum_mod3 == 0U;
}

/* Keep the original 14-character input format, but avoid i % 7. */
static int parse_state(const char *input, state_t *state)
{
    for (uint8_t i = 0; i < CUBIES; ++i) {
        if (input[i] < '1' || input[i] > '7')
            return 0;
        state->p[i] = (uint8_t)(input[i] - '1');
    }
    for (uint8_t i = 0; i < CUBIES; ++i) {
        char c = input[CUBIES + i];
        if (c < '1' || c > '3')
            return 0;
        state->o[i] = (uint8_t)(c - '1');
    }
    return input[14] == '\0' && valid(state);
}

/* ---------- Stage 2 table construction, now RV32I-friendly ---------- */
static void build_transition_tables(void)
{
    state_t state;
    for (uint16_t rank = 0; rank < PERMUTATIONS; ++rank) { //permutation_next[face][rank] index為face, rank，value為做了face動作的rank
        unrank_permutation(rank, &state);
        for (uint8_t face = 0; face < 3U; ++face) {
            state_t next = quarter_turn(state, face);
            permutation_next[face][rank] = rank_permutation(&next);
        }
    }
    for (uint16_t rank = 0; rank < ORIENTATIONS; ++rank) {
        unrank_orientation(rank, &state);
        for (uint8_t face = 0; face < 3U; ++face) {
            state_t next = quarter_turn(state, face);
            orientation_next[face][rank] = rank_orientation(&next);
        }
    }
}

static int build_permutation_pdb(void)
{
    uint16_t head = 0, tail = 0;
    for (uint16_t i = 0; i < PERMUTATIONS; ++i)
        perm_dist[i] = UINT8_MAX;
    perm_dist[0] = 0;
    pdb_queue[tail++] = 0;
    while (head < tail) {
        uint16_t here = pdb_queue[head++];
        for (uint8_t face = 0; face < 3U; ++face) {
            uint16_t next = here;
            for (uint8_t turn = 0; turn < 3U; ++turn) {
                next = permutation_next[face][next];
                if (perm_dist[next] == UINT8_MAX) {
                    perm_dist[next] = (uint8_t)(perm_dist[here] + 1U);
                    pdb_queue[tail++] = next;
                }
            }
        }
    }
    return tail == PERMUTATIONS;
}

static int build_orientation_pdb(void)
{
    uint16_t head = 0, tail = 0;
    for (uint16_t i = 0; i < ORIENTATIONS; ++i)
        ori_dist[i] = UINT8_MAX;
    ori_dist[0] = 0;
    pdb_queue[tail++] = 0;
    while (head < tail) {
        uint16_t here = pdb_queue[head++];
        for (uint8_t face = 0; face < 3U; ++face) {
            uint16_t next = here;
            for (uint8_t turn = 0; turn < 3U; ++turn) {
                next = orientation_next[face][next];
                if (ori_dist[next] == UINT8_MAX) {
                    ori_dist[next] = (uint8_t)(ori_dist[here] + 1U);
                    pdb_queue[tail++] = next;
                }
            }
        }
    }
    return tail == ORIENTATIONS;
}

/* Search uses the same Stage 2 heuristic, but keeps the two coordinates directly. */
static uint8_t heuristic(uint16_t p, uint16_t o)
{
    //在ida_solver階段就將rank分好了，直接用p,o查表
    uint8_t hp = perm_dist[p];
    uint8_t ho = ori_dist[o];
    return hp > ho ? hp : ho;
}

/* Select a transition row without multiplying a variable face by a large stride. */
static uint16_t permutation_turn(uint16_t p, uint8_t face)
{
    //分配去已建好表的找下一個rank
    if (face == 0U)
        return permutation_next[0][p];
    if (face == 1U)
        return permutation_next[1][p];
    return permutation_next[2][p];
}

static uint16_t orientation_turn(uint16_t o, uint8_t face)
{
    if (face == 0U)
        return orientation_next[0][o];
    if (face == 1U)
        return orientation_next[1][o];
    return orientation_next[2][o];
}

//transition tables 已經告訴我們 move 後 coordinate 會去哪，直接查表

static void apply_coordinate_move(uint16_t p,
                                  uint16_t o,
                                  uint8_t move,
                                  uint16_t *next_p,
                                  uint16_t *next_o)
{
    uint8_t face = move_face[move];
    uint8_t turns = move_turns[move];
    for (uint8_t i = 0; i < turns; ++i) {
        //傳rank 和 要轉的面向face，再要轉的次數(turn)去做loop
        p = permutation_turn(p, face);
        o = orientation_turn(o, face);
    }
    *next_p = p;
    *next_o = o;
}

/*
 * One IDA* bound using an explicit fixed-size stack.
 * This replaces Stage 2 recursion because the target forbids recursion.
 */
static int search_bound(uint16_t start_p,   //起點的 permutation rank
                        uint16_t start_o,   //起點的 orientation rank
                        uint8_t bound,
                        uint8_t path[MAX_DEPTH],    //保存目前找到的 moves
                        uint8_t *solution_length,   //如果找到答案，把答案長度傳出去。
                        uint8_t *next_bound)    //記錄超過目前 bound 的 f 中最小的那一個
{   //stack array
    //MAX_DEPTH + 1 因為depth 包含起點depth 0 ，還可以走11步，代表12state slot
    uint16_t p_stack[MAX_DEPTH + 1];    //這一層 cube 的 permutation rank，ex:p_stack[0] = 起點 permutation，p_stack[1] = 做第一步之後的rank
    uint16_t o_stack[MAX_DEPTH + 1];    //表示這一層 orientation rank
    uint8_t previous_face[MAX_DEPTH + 1];   //用哪個 face 的 move 來到這一層的
    uint8_t next_move[MAX_DEPTH + 1];   //如果之後回到這一層，下一個應該繼續試哪個 move
    uint8_t entered[MAX_DEPTH + 1]; //這一層是不是已經做過「第一次進入 node 時要做的事情」，不用再做一些算 heuristic，算 f等等的是
    uint8_t depth = 0;
    p_stack[0] = start_p;
    o_stack[0] = start_o;
    previous_face[0] = NO_FACE;//合法 face為 0 1 2
    entered[0] = 0;
    for (;;) {//無限迴圈
        if (!entered[depth]) {
            uint8_t h = heuristic(p_stack[depth], o_stack[depth]);
            uint8_t f = (uint8_t)(depth + h);
            if (f > bound) {
                if (f < *next_bound)
                    *next_bound = f;//next_bound
                if (depth == 0U)
                    return 0;
                --depth;//正常 pruning 要 backtrack
                continue;
            }
            //permutation rank 0，orientation rank 0就是 solved cube。
            if (p_stack[depth] == 0U && o_stack[depth] == 0U) {
                *solution_length = depth;
                return 1;
            }
            //已經到 depth 11
            if (depth == MAX_DEPTH) {
                if (depth == 0U)
                    return 0;
                --depth;
                continue;
            }
            next_move[depth] = 0;//這個 node 接下來從 move 0 = R 開始試。
            entered[depth] = 1;//這個 node 的第一次 evaluation 已經做完。
        }
        //Same-face pruning
        while (next_move[depth] < MOVES &&
               move_face[next_move[depth]] == previous_face[depth]) {//same move
            ++next_move[depth];//skip
        }
        if (next_move[depth] >= MOVES) {//全部都處理完九種都是過了
            entered[depth] = 0;//把這層清掉。因為未來如果另一條 path 又走到這個 stack slot，需要把它當作新的 node。
            if (depth == 0U)
                return 0;//整棵目前 bound 的搜尋樹全部找完，都沒有答案，所以這輪失敗
            --depth;//否則，回 parent
            continue;
        }
        uint8_t move = next_move[depth]++;//現在要試 move，但先記住「下次回到這裡要試 move+1的move」。
        uint16_t child_p, child_o;//move後的下一個 state
        apply_coordinate_move(p_stack[depth], //拿parent的p,o rank 和move來查表
                              o_stack[depth],
                              move,
                              &child_p,//直接改小孩的p,o rank;
                              &child_o);
        path[depth] = move; //把這次的move放進path裡
        ++depth; //到下一個depth了 開始是child state會不會找到solved state
        p_stack[depth] = child_p;
        o_stack[depth] = child_o;
        previous_face[depth] = move_face[move]; //把這個move動作放進previous_face[depth]紀錄
        entered[depth] = 0;//開關打開 從新初始化及檢查
    }
}

static int ida_solve(state_t start,
                     uint8_t path[MAX_DEPTH],
                     uint8_t *solution_length)
{
    //只做一次 rank分配
    uint16_t start_p = rank_permutation(&start);
    uint16_t start_o = rank_orientation(&start);
    uint8_t bound = heuristic(start_p, start_o);
    while (bound <= MAX_DEPTH) {
        uint8_t next_bound = UINT8_MAX;
        if (search_bound(start_p,
                         start_o,
                         bound,
                         path,
                         solution_length,
                         &next_bound)) {
            return 1;
        }
        if (next_bound == UINT8_MAX)
            return 0;
        bound = next_bound;
    }
    return 0;
}

static int is_solved(const state_t *state)
{
    for (uint8_t i = 0; i < CUBIES; ++i) {
        if (state->p[i] != i || state->o[i] != 0U)
            return 0;
    }
    return 1;
}

/* Required end-to-end validation: replay the returned path inside the program. */
static int validate_solution(state_t start,
                             const uint8_t path[MAX_DEPTH],
                             uint8_t length)
{
    for (uint8_t i = 0; i < length; ++i)
        start = apply_move(start, path[i]);
    return is_solved(&start);
}

static int verify_pdbs(void)
{
    uint8_t perm_max = 0;
    uint8_t ori_max = 0;
    if (perm_dist[0] != 0U || ori_dist[0] != 0U)
        return 0;
    for (uint16_t i = 0; i < PERMUTATIONS; ++i) {
        if (perm_dist[i] == UINT8_MAX)
            return 0;
        if (perm_dist[i] > perm_max)
            perm_max = perm_dist[i];
    }
    for (uint16_t i = 0; i < ORIENTATIONS; ++i) {
        if (ori_dist[i] == UINT8_MAX)
            return 0;
        if (ori_dist[i] > ori_max)
            ori_max = ori_dist[i];
    }
    return perm_max == 7U && ori_max == 6U;
}

static int run_test(const char *input, uint8_t expected_length)
{
    state_t state;
    uint8_t path[MAX_DEPTH];
    uint8_t length = 0;
    if (!parse_state(input, &state))
        return 0;
    if (!ida_solve(state, path, &length))
        return 0;
    if (length != expected_length)
        return 0;
    return validate_solution(state, path, length);
}

static int self_test(void)
{
    const state_t solved = {{0, 1, 2, 3, 4, 5, 6}, {0}};
    for (uint8_t move = 0; move < MOVES; ++move) {
        state_t state = apply_move(solved, move);
        state = apply_move(state, inverse_move[move]);
        if (!is_solved(&state))
            return 0;
    }
    if (!verify_pdbs())
        return 0;
    if (!run_test("12345671111111", 0U))
        return 0;
    if (!run_test("25314672313211", 1U))
        return 0;
    if (!run_test("21345671111111", 11U))
        return 0;
    return 1;
}

static int output_failed(void)
{
    return fflush(stdout) != 0 || ferror(stdout);
}

int main(int argc, char **argv)
{
    state_t state;
    /* Keep the Stage 2 behavior: build transitions and PDBs at run time. */
    build_transition_tables();
    if (!build_permutation_pdb() || !build_orientation_pdb()) {
        fputs("could not build PDBs\n", stderr);
        return 1;
    }
    if (argc == 2 && !strcmp(argv[1], "--self-test")) {
        if (!self_test()) {
            fputs("self-test failed\n", stderr);
            return 1;
        }
        puts("self-test passed");
        return output_failed();
    }
    if (argc != 2 || !parse_state(argv[1], &state)) {
        fprintf(stderr,
                "usage: %s PPPPPPPOOOOOOO\n",
                argc > 0 && argv[0] ? argv[0] : "solver_stage3_runtime");
        return 2;
    }
    uint8_t path[MAX_DEPTH];
    uint8_t solution_length = 0;
    if (!ida_solve(state, path, &solution_length)) {
        fputs("no solution found\n", stderr);
        return 1;
    }
    if (!validate_solution(state, path, solution_length)) {
        fputs("internal solution validation failed\n", stderr);
        return 1;
    }
#ifndef NO_RENDER
    for (uint8_t i = 0; i < solution_length; ++i) {
        if (i != 0U)
            putchar(' ');
        fputs(move_names[path[i]], stdout);
    }
    putchar('\n');
#endif
    return output_failed();
}

