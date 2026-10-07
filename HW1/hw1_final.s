# Stage 4 - RV32I Rubik's Cube Solver
# Final version

.equ CUBIES,       7
.equ PERMUTATIONS, 5040
.equ ORIENTATIONS, 729
.equ MOVES,         9
.equ MAX_DEPTH,    11
.equ NO_FACE,       3

.data

# 14-character input
input_state:
    # "21345671111111"
    .byte 50, 49, 51, 52, 53, 54, 55
    .byte 49, 49, 49, 49, 49, 49, 49
    .byte 0
    .zero 1

# 3 的次方表
power3_table:
    .word 243, 81, 27, 9, 3, 1

# permutation unrank 用的 factorial
factorial_table:
    .word 720, 120, 24, 6, 2, 1, 1

# R/B/D quarter-turn source
source:
    .byte 1, 4, 2, 0, 3, 5, 6
    .byte 0, 1, 2, 4, 5, 6, 3
    .byte 0, 2, 5, 3, 1, 4, 6

# R/B/D orientation change
twist:
    .byte 1, 2, 0, 2, 1, 0, 0
    .byte 0, 0, 0, 1, 2, 1, 2
    .byte 0, 0, 0, 0, 0, 0, 0

# permutation transition table
permutation_next:
    .zero 30240

# orientation transition table
orientation_next:
    .zero 4374

# permutation PDB
perm_dist:
    .zero 5040

# orientation PDB
ori_dist:
    .zero 729

    .zero 1

# PDB BFS queue
pdb_queue:
    .zero 10080

# IDA* stacks
p_stack:
    .zero 24

o_stack:
    .zero 24

previous_face:
    .zero 12

next_move:
    .zero 12

entered:
    .zero 12

path:
    .zero 11

# parsed p[0..6], o[0..6]
parsed_state:
    .zero 14

# Code
.text
.globl main

# main: parse -> build tables/PDB -> solve -> validate
main:
    la a0, input_state
    la a1, parsed_state
    call parse_input
    la a0, parsed_state
    call rank_permutation
    mv s0, a0
    la a0, parsed_state
    call rank_orientation
    mv s1, a0
    call build_transition_tables
    call build_permutation_pdb
    call build_orientation_pdb
    mv a0, s0
    mv a1, s1
    call ida_solve
    mv s2, a0
    mv s3, a1
    mv a0, s0
    mv a1, s1
    mv a2, s3
    call validate_solution
    mv s4, a0
    li a7, 10
    ecall

# quarter turn
quarter_turn:
    slli t0, a1, 3
    sub  t0, t0, a1
    la   t1, source
    add  t1, t1, t0
    la   t2, twist
    add  t2, t2, t0
    li t3, 0
    li a3, 7
    li a4, 3

quarter_turn_loop:
    bge t3, a3, quarter_turn_done
    add t4, t1, t3
    lbu t5, 0(t4)
    add t4, a0, t5
    lbu t6, 0(t4)
    add t4, a2, t3
    sb t6, 0(t4)
    add  t4, a0, t5
    addi t4, t4, 7
    lbu t6, 0(t4)
    add t4, t2, t3
    lbu t4, 0(t4)
    add t6, t6, t4
    blt t6, a4, quarter_turn_orientation_ready
    addi t6, t6, -3

quarter_turn_orientation_ready:
    add  t4, a2, t3
    addi t4, t4, 7
    sb t6, 0(t4)
    addi t3, t3, 1
    j quarter_turn_loop

quarter_turn_done:
    ret

# 小常數乘法，避免 MUL
multiply_small:
    li   t0, 7
    beq  a1, t0, multiply_7
    li   t0, 6
    beq  a1, t0, multiply_6
    li   t0, 5
    beq  a1, t0, multiply_5
    li   t0, 4
    beq  a1, t0, multiply_4
    li   t0, 3
    beq  a1, t0, multiply_3
    li   t0, 2
    beq  a1, t0, multiply_2
    ret

multiply_7:
    slli t0, a0, 3
    sub  a0, t0, a0
    ret

multiply_6:
    slli t0, a0, 2
    slli t1, a0, 1
    add  a0, t0, t1
    ret

multiply_5:
    slli t0, a0, 2
    add  a0, t0, a0
    ret

multiply_4:
    slli a0, a0, 2
    ret

multiply_3:
    slli t0, a0, 1
    add  a0, t0, a0
    ret

multiply_2:
    slli a0, a0, 1
    ret

# permutation rank
rank_permutation:
    addi sp, sp, -32
    sw ra, 28(sp)
    sw s0, 24(sp)
    sw s1, 20(sp)
    sw s2, 16(sp)
    sw s3, 12(sp)
    mv s0, a0
    li s1, 0
    li s2, 0

rank_permutation_outer_loop:
    li t0, 7
    bge s2, t0, rank_permutation_done
    li s3, 0
    addi t1, s2, 1
    add t2, s0, s2
    lbu t3, 0(t2)

rank_permutation_inner_loop:
    li t0, 7
    bge t1, t0, rank_permutation_after_inner
    add t2, s0, t1
    lbu t4, 0(t2)
    bgeu t4, t3, rank_permutation_not_smaller
    addi s3, s3, 1

rank_permutation_not_smaller:
    addi t1, t1, 1
    j rank_permutation_inner_loop

rank_permutation_after_inner:
    mv a0, s1
    li t0, 7
    sub a1, t0, s2
    call multiply_small
    add s1, a0, s3
    addi s2, s2, 1
    j rank_permutation_outer_loop

rank_permutation_done:
    mv a0, s1
    lw s3, 12(sp)
    lw s2, 16(sp)
    lw s1, 20(sp)
    lw s0, 24(sp)
    lw ra, 28(sp)
    addi sp, sp, 32
    ret

# orientation rank
rank_orientation:
    mv   t0, a0
    addi t0, t0, 7
    li   t1, 0
    li   t2, 0
    li   t3, 6

rank_orientation_loop:
    bge  t2, t3, rank_orientation_done
    lbu  t4, 0(t0)
    slli t5, t1, 1
    add  t1, t5, t1
    add  t1, t1, t4
    addi t0, t0, 1
    addi t2, t2, 1
    j rank_orientation_loop

rank_orientation_done:
    mv a0, t1
    ret

# permutation unrank
unrank_permutation:
    addi sp, sp, -16
    mv t0, a1
    mv t1, sp
    li t2, 0

unrank_permutation_available_loop:
    li t3, 7
    bge t2, t3, unrank_permutation_available_done
    add t4, t1, t2
    sb t2, 0(t4)
    addi t2, t2, 1
    j unrank_permutation_available_loop

unrank_permutation_available_done:
    la t3, factorial_table
    li t2, 0

unrank_permutation_outer_loop:
    li t4, 7
    bge t2, t4, unrank_permutation_orientation
    lw t4, 0(t3)
    li t5, 0

unrank_permutation_div_loop:
    bltu a0, t4, unrank_permutation_div_done
    sub a0, a0, t4
    addi t5, t5, 1
    j unrank_permutation_div_loop

unrank_permutation_div_done:
    add t6, t1, t5
    lbu t6, 0(t6)
    add a2, t0, t2
    sb t6, 0(a2)
    mv a2, t5
    li a3, 6
    sub a3, a3, t2

unrank_permutation_shift_loop:
    bge a2, a3, unrank_permutation_shift_done
    add t6, t1, a2
    lbu a4, 1(t6)
    sb a4, 0(t6)
    addi a2, a2, 1
    j unrank_permutation_shift_loop

unrank_permutation_shift_done:
    addi t3, t3, 4
    addi t2, t2, 1
    j unrank_permutation_outer_loop

unrank_permutation_orientation:
    addi t4, t0, 7
    li t2, 0

unrank_permutation_orientation_loop:
    li t5, 7
    bge t2, t5, unrank_permutation_done
    sb zero, 0(t4)
    addi t4, t4, 1
    addi t2, t2, 1
    j unrank_permutation_orientation_loop

unrank_permutation_done:
    addi sp, sp, 16
    ret

# orientation unrank
unrank_orientation:
    mv t0, a1
    li t1, 0
    li t2, 7

unrank_orientation_perm_loop:
    bge t1, t2, unrank_orientation_perm_done
    sb t1, 0(t0)
    addi t0, t0, 1
    addi t1, t1, 1
    j unrank_orientation_perm_loop

unrank_orientation_perm_done:
    la t0, power3_table
    addi t1, a1, 7
    li t2, 0
    li t3, 0

unrank_orientation_loop:
    li t4, 6
    bge t2, t4, unrank_orientation_last
    lw t4, 0(t0)
    li t5, 0
    slli t6, t4, 1
    bgeu a0, t6, unrank_orientation_value_2
    bgeu a0, t4, unrank_orientation_value_1
    j unrank_orientation_store

unrank_orientation_value_2:
    sub a0, a0, t6
    li t5, 2
    j unrank_orientation_store

unrank_orientation_value_1:
    sub a0, a0, t4
    li t5, 1

unrank_orientation_store:
    sb t5, 0(t1)
    add t3, t3, t5
    li t6, 3
    blt t3, t6, unrank_orientation_sum_done
    addi t3, t3, -3

unrank_orientation_sum_done:
    addi t0, t0, 4
    addi t1, t1, 1
    addi t2, t2, 1
    j unrank_orientation_loop

unrank_orientation_last:
    beq t3, zero, unrank_orientation_last_zero
    li t4, 3
    sub t4, t4, t3
    sb t4, 0(t1)
    ret

unrank_orientation_last_zero:
    sb zero, 0(t1)
    ret

# 建立 permutation/orientation transition tables
build_transition_tables:
    addi sp, sp, -64
    sw s7, 28(sp)
    sw s6, 32(sp)
    sw s5, 36(sp)
    sw s4, 40(sp)
    sw s3, 44(sp)
    sw s2, 48(sp)
    sw s1, 52(sp)
    sw s0, 56(sp)
    sw ra, 60(sp)
    mv s2, sp
    addi s3, sp, 14
    la s4, permutation_next
    la s5, orientation_next
    li s6, PERMUTATIONS
    li s7, ORIENTATIONS
    li s0, 0

build_transition_face_loop:
    li t0, 3
    bge s0, t0, build_transition_done
    li s1, 0

build_permutation_loop:
    bge s1, s6, build_orientation_start
    mv a0, s1
    mv a1, s2
    call unrank_permutation
    mv a0, s2
    mv a1, s0
    mv a2, s3
    call quarter_turn
    mv a0, s3
    call rank_permutation
    sh a0, 0(s4)
    addi s4, s4, 2
    addi s1, s1, 1
    j build_permutation_loop

build_orientation_start:
    li s1, 0

build_orientation_loop:
    bge s1, s7, build_transition_next_face
    mv a0, s1
    mv a1, s2
    call unrank_orientation
    mv a0, s2
    mv a1, s0
    mv a2, s3
    call quarter_turn
    mv a0, s3
    call rank_orientation
    sh a0, 0(s5)
    addi s5, s5, 2
    addi s1, s1, 1
    j build_orientation_loop

build_transition_next_face:
    addi s0, s0, 1
    j build_transition_face_loop

build_transition_done:
    lw s7, 28(sp)
    lw s6, 32(sp)
    lw s5, 36(sp)
    lw s4, 40(sp)
    lw s3, 44(sp)
    lw s2, 48(sp)
    lw s1, 52(sp)
    lw s0, 56(sp)
    lw ra, 60(sp)
    addi sp, sp, 64
    ret

# 建立 permutation PDB
build_permutation_pdb:
    addi sp, sp, -32
    sw s0,  0(sp)
    sw s1,  4(sp)
    sw s2,  8(sp)
    sw s3, 12(sp)
    sw s4, 16(sp)
    sw s5, 20(sp)
    sw s6, 24(sp)
    sw s7, 28(sp)
    la s0, perm_dist
    mv t0, s0
    li t1, PERMUTATIONS
    li t2, 255

build_perm_pdb_init_loop:
    beq t1, zero, build_perm_pdb_init_done
    sb t2, 0(t0)
    addi t0, t0, 1
    addi t1, t1, -1
    j build_perm_pdb_init_loop

build_perm_pdb_init_done:
    sb zero, 0(s0)
    la s1, pdb_queue
    mv s2, s1
    sh zero, 0(s2)
    addi s2, s2, 2
    la s3, permutation_next
    li s4, 10080
    li t4, 255
    li t5, 3

build_perm_pdb_bfs_loop:
    bgeu s1, s2, build_perm_pdb_done
    lhu s5, 0(s1)
    addi s1, s1, 2
    add t0, s0, s5
    lbu s6, 0(t0)
    addi s6, s6, 1
    li t0, 0
    mv s7, s3

build_perm_pdb_face_loop:
    bge t0, t5, build_perm_pdb_bfs_loop
    mv t2, s5
    li t1, 0

build_perm_pdb_turn_loop:
    bge t1, t5, build_perm_pdb_next_face
    slli t3, t2, 1
    add t3, s7, t3
    lhu t2, 0(t3)
    add t3, s0, t2
    lbu t6, 0(t3)
    bne t6, t4, build_perm_pdb_already_seen
    sb s6, 0(t3)
    sh t2, 0(s2)
    addi s2, s2, 2

build_perm_pdb_already_seen:
    addi t1, t1, 1
    j build_perm_pdb_turn_loop

build_perm_pdb_next_face:
    add s7, s7, s4
    addi t0, t0, 1
    j build_perm_pdb_face_loop

build_perm_pdb_done:
    lw s0,  0(sp)
    lw s1,  4(sp)
    lw s2,  8(sp)
    lw s3, 12(sp)
    lw s4, 16(sp)
    lw s5, 20(sp)
    lw s6, 24(sp)
    lw s7, 28(sp)
    addi sp, sp, 32
    ret

# 建立 orientation PDB
build_orientation_pdb:
    addi sp, sp, -32
    sw s0,  0(sp)
    sw s1,  4(sp)
    sw s2,  8(sp)
    sw s3, 12(sp)
    sw s4, 16(sp)
    sw s5, 20(sp)
    sw s6, 24(sp)
    sw s7, 28(sp)
    la s0, ori_dist
    mv t0, s0
    li t1, ORIENTATIONS
    li t2, 255

build_ori_pdb_init_loop:
    beq t1, zero, build_ori_pdb_init_done
    sb t2, 0(t0)
    addi t0, t0, 1
    addi t1, t1, -1
    j build_ori_pdb_init_loop

build_ori_pdb_init_done:
    sb zero, 0(s0)
    la s1, pdb_queue
    mv s2, s1
    sh zero, 0(s2)
    addi s2, s2, 2
    la s3, orientation_next
    li s4, 1458
    li t4, 255
    li t5, 3

build_ori_pdb_bfs_loop:
    bgeu s1, s2, build_ori_pdb_done
    lhu s5, 0(s1)
    addi s1, s1, 2
    add t0, s0, s5
    lbu s6, 0(t0)
    addi s6, s6, 1
    li t0, 0
    mv s7, s3

build_ori_pdb_face_loop:
    bge t0, t5, build_ori_pdb_bfs_loop
    mv t2, s5
    li t1, 0

build_ori_pdb_turn_loop:
    bge t1, t5, build_ori_pdb_next_face
    slli t3, t2, 1
    add t3, s7, t3
    lhu t2, 0(t3)
    add t3, s0, t2
    lbu t6, 0(t3)
    bne t6, t4, build_ori_pdb_already_seen
    sb s6, 0(t3)
    sh t2, 0(s2)
    addi s2, s2, 2

build_ori_pdb_already_seen:
    addi t1, t1, 1
    j build_ori_pdb_turn_loop

build_ori_pdb_next_face:
    add s7, s7, s4
    addi t0, t0, 1
    j build_ori_pdb_face_loop

build_ori_pdb_done:
    lw s0,  0(sp)
    lw s1,  4(sp)
    lw s2,  8(sp)
    lw s3, 12(sp)
    lw s4, 16(sp)
    lw s5, 20(sp)
    lw s6, 24(sp)
    lw s7, 28(sp)
    addi sp, sp, 32
    ret

# h = max(perm_dist[p], ori_dist[o])
heuristic:
    la t0, perm_dist
    add t0, t0, a0
    lbu t1, 0(t0)
    la t0, ori_dist
    add t0, t0, a1
    lbu t2, 0(t0)
    bgeu t1, t2, heuristic_use_perm
    mv a0, t2
    ret

heuristic_use_perm:
    mv a0, t1
    ret

# coordinate move: 0..2=R, 3..5=B, 6..8=D
coordinate_move:
    la t2, permutation_next
    la t3, orientation_next
    li t0, 3
    blt a2, t0, coordinate_move_R
    li t0, 6
    blt a2, t0, coordinate_move_B

coordinate_move_D:
    addi t4, a2, -5
    li t5, 20160
    add t2, t2, t5
    li t5, 2916
    add t3, t3, t5
    j coordinate_move_loop

coordinate_move_B:
    addi t4, a2, -2
    li t5, 10080
    add t2, t2, t5
    li t5, 1458
    add t3, t3, t5
    j coordinate_move_loop

coordinate_move_R:
    addi t4, a2, 1

coordinate_move_loop:
    beq t4, zero, coordinate_move_done
    slli t5, a0, 1
    add t6, t2, t5
    lhu a0, 0(t6)
    slli t5, a1, 1
    add t6, t3, t5
    lhu a1, 0(t6)
    addi t4, t4, -1
    j coordinate_move_loop

coordinate_move_done:
    ret

# bounded IDA* search
search_bound:
    addi sp, sp, -48
    sw s0,   0(sp)
    sw s1,   4(sp)
    sw s2,   8(sp)
    sw s3,  12(sp)
    sw s4,  16(sp)
    sw s5,  20(sp)
    sw s6,  24(sp)
    sw s7,  28(sp)
    sw s8,  32(sp)
    sw s9,  36(sp)
    sw s10, 40(sp)
    sw ra,  44(sp)
    li s0, 0
    mv s1, a2
    li s2, 255
    la s3, p_stack
    la s4, o_stack
    la s5, previous_face
    la s6, next_move
    la s7, entered
    la s8, path
    sh a0, 0(s3)
    sh a1, 0(s4)
    li t0, NO_FACE
    sb t0, 0(s5)
    sb zero, 0(s7)

search_bound_loop:
    add t0, s7, s0
    lbu t1, 0(t0)
    bne t1, zero, search_bound_choose_move
    slli t0, s0, 1
    add t1, s3, t0
    lhu t2, 0(t1)
    add t1, s4, t0
    lhu t3, 0(t1)
    la t0, perm_dist
    add t0, t0, t2
    lbu t2, 0(t0)
    la t0, ori_dist
    add t0, t0, t3
    lbu t3, 0(t0)
    bgeu t2, t3, search_bound_inline_h_ready
    mv t2, t3

search_bound_inline_h_ready:
    add t0, s0, t2
    bgeu s1, t0, search_bound_within_bound
    bgeu t0, s2, search_bound_pruned
    mv s2, t0

search_bound_pruned:
    beq s0, zero, search_bound_failed
    addi s0, s0, -1
    j search_bound_loop

search_bound_within_bound:
    slli t0, s0, 1
    add t1, s3, t0
    lhu t2, 0(t1)
    bne t2, zero, search_bound_not_solved
    add t1, s4, t0
    lhu t2, 0(t1)
    beq t2, zero, search_bound_found

search_bound_not_solved:
    li t0, MAX_DEPTH
    bgeu s0, t0, search_bound_backtrack
    add t0, s6, s0
    sb zero, 0(t0)
    add t0, s7, s0
    li t1, 1
    sb t1, 0(t0)

search_bound_choose_move:
    add t0, s6, s0
    lbu t1, 0(t0)
    li t2, MOVES
    bgeu t1, t2, search_bound_exhausted
    li t2, 3
    bltu t1, t2, search_bound_face_R
    li t2, 6
    bltu t1, t2, search_bound_face_B
    li t3, 2
    j search_bound_face_checked

search_bound_face_R:
    li t3, 0
    j search_bound_face_checked

search_bound_face_B:
    li t3, 1

search_bound_face_checked:
    add t0, s5, s0
    lbu t2, 0(t0)
    beq t2, t3, search_bound_skip_move
    mv s9, t1
    mv s10, t3
    add t0, s6, s0
    addi t1, t1, 1
    sb t1, 0(t0)
    slli t0, s0, 1
    add t1, s3, t0
    lhu a0, 0(t1)
    add t1, s4, t0
    lhu a1, 0(t1)
    mv a2, s9
    call coordinate_move
    add t0, s8, s0
    sb s9, 0(t0)
    addi s0, s0, 1
    slli t0, s0, 1
    add t1, s3, t0
    sh a0, 0(t1)
    add t1, s4, t0
    sh a1, 0(t1)
    add t1, s5, s0
    sb s10, 0(t1)
    add t1, s7, s0
    sb zero, 0(t1)
    j search_bound_loop

search_bound_skip_move:
    addi t1, t1, 1
    add t0, s6, s0
    sb t1, 0(t0)
    j search_bound_choose_move

search_bound_exhausted:
    add t0, s7, s0
    sb zero, 0(t0)

search_bound_backtrack:
    beq s0, zero, search_bound_failed
    addi s0, s0, -1
    j search_bound_loop

search_bound_found:
    li a0, 1
    mv a1, s0
    mv a2, s2
    j search_bound_return

search_bound_failed:
    li a0, 0
    li a1, 0
    mv a2, s2

search_bound_return:
    lw s0,   0(sp)
    lw s1,   4(sp)
    lw s2,   8(sp)
    lw s3,  12(sp)
    lw s4,  16(sp)
    lw s5,  20(sp)
    lw s6,  24(sp)
    lw s7,  28(sp)
    lw s8,  32(sp)
    lw s9,  36(sp)
    lw s10, 40(sp)
    lw ra,  44(sp)
    addi sp, sp, 48
    ret

# IDA* solve
ida_solve:
    addi sp, sp, -16
    sw s0,  0(sp)
    sw s1,  4(sp)
    sw s2,  8(sp)
    sw ra, 12(sp)
    mv s0, a0
    mv s1, a1
    mv a0, s0
    mv a1, s1
    call heuristic
    mv s2, a0

ida_solve_loop:
    li t0, MAX_DEPTH
    bltu t0, s2, ida_solve_failed
    mv a0, s0
    mv a1, s1
    mv a2, s2
    call search_bound
    bne a0, zero, ida_solve_found
    mv s2, a2
    j ida_solve_loop

ida_solve_found:
    j ida_solve_return

ida_solve_failed:
    li a0, 0
    li a1, 0

ida_solve_return:
    lw s0,  0(sp)
    lw s1,  4(sp)
    lw s2,  8(sp)
    lw ra, 12(sp)
    addi sp, sp, 16
    ret

# 重新套用 path，確認最後回到 solved
validate_solution:
    addi sp, sp, -32
    sw s0,  8(sp)
    sw s1, 12(sp)
    sw s2, 16(sp)
    sw s3, 20(sp)
    sw s4, 24(sp)
    sw ra, 28(sp)
    mv s0, a0
    mv s1, a1
    li s2, 0
    mv s3, a2
    la s4, path

validate_solution_loop:
    bgeu s2, s3, validate_solution_check
    add t0, s4, s2
    lbu t1, 0(t0)
    mv a0, s0
    mv a1, s1
    mv a2, t1
    call coordinate_move
    mv s0, a0
    mv s1, a1
    addi s2, s2, 1
    j validate_solution_loop

validate_solution_check:
    bne s0, zero, validate_solution_invalid
    bne s1, zero, validate_solution_invalid
    li a0, 1
    j validate_solution_return

validate_solution_invalid:
    li a0, 0

validate_solution_return:
    lw s0,  8(sp)
    lw s1, 12(sp)
    lw s2, 16(sp)
    lw s3, 20(sp)
    lw s4, 24(sp)
    lw ra, 28(sp)
    addi sp, sp, 32
    ret

# 14-character input -> internal state
parse_input:
    mv t0, a0
    mv t1, a1
    li t2, 0
    li t4, 7

parse_input_permutation_loop:
    bge t2, t4, parse_input_orientation_start
    lbu t3, 0(t0)
    addi t3, t3, -49
    sb t3, 0(t1)
    addi t0, t0, 1
    addi t1, t1, 1
    addi t2, t2, 1
    j parse_input_permutation_loop

parse_input_orientation_start:
    li t2, 0

parse_input_orientation_loop:
    bge t2, t4, parse_input_done
    lbu t3, 0(t0)
    addi t3, t3, -49
    sb t3, 0(t1)
    addi t0, t0, 1
    addi t1, t1, 1
    addi t2, t2, 1
    j parse_input_orientation_loop

parse_input_done:
    ret
