# 32-bit_Risc_Processor

Design of a 32-bit Processor using ARM instructions. Design includes controller, datapath, data memory, and instruction memory.

## Table of contents

[Instruction Sets & Microarchitecture]()
[Schematics]()
[Verilog Code]()
[Verifcation of CPU]()
[Conclusion]()


## Instruction Sets / Microarchitecture

### Instruction Register

| 31:28 | 27:26 | 25:20 | 19:16 | 15:12 | 11:0 |
|-------|-------|-------|-------|-------|------|
| cond  | op    | funct | Rn    | Rd    | Src2 |

which can be simplified to

| 31:28 | 27:26 | 25 | 24:21| 20| 19:16 | 15:12 | 11:0 |
|-------|-------|----|------|---|-------|-------|------|
| cond  | op    | I  |funct | S | Rn    | Rd    | Src2 |

The tables below describe the different types of operations, mnemonics, and the type of instruction being executed.

| cond | mnemonic | name          | condEx |
|------|----------|---------------|--------|
| 0000 | EQ       | Equal         | Z      |
| 0001 | NE       | NotEqual      | ~Z     |
| 0100 | MI       | Minus/Negative| N      |
| 1110 | AL       | Unconditional | Ignored|

| cmd | operation |
|-----|-----------|
| 0000| AND       |
| 0100| ADD       |
| 0010| SUB       |
| 1100| ORR       |

| op | instruction type  |
|----|-------------------|
| 00 | Data-Processing   |
| 01 | Memory-Instruction|
| 10 | Branch-Instruction|

### Data Proccessing

The data-processing instruction type is used for register and immediate valued operations.

| 31:28 | 27:26 | 25 | 24:21 | 20 | 19:16 | 15:12 | 11:0 |
|-------|-------|----|-------|----|-------|-------|------|
| cond  | 00    | I  | cmd   | S  | Rn    | Rd    | Src2 |

If I equals 1, then it means an immediate value is being used. Thus, the table below is used for forming the proper instruction.

| 11:8 | 7:0 |
|------|-----|
| rot  | imm8|

If I equals 0, then it indicates a register value is being used. Thus, the table below is used for forming the proper instruction,

| 11:7 | 6:5 | 4 | 3:0 |
|------|-----|---|-----|
| shamt| sh  | 0 | Rm  |

where the table below is used for describing the type of shift operation.

| instruction | sh | operation              |
|-------------|----|------------------------|
| LSL         | 00 | Logical Shift Left     |
| LSR         | 01 | Logical Shift Right    |
| ASR         | 10 | Arithmetic Shift Right |
| ROR         | 11 | Rotate Right           |

### Memory Instruction

The memory-instruction type is used for loading and storing operations (LDR/STR).

| 31:28 | 27:26 | 25 | 24 | 23 | 22 | 21 | 20 | 19:16 | 15:12 | 11:0 |
|-------|-------|----|----|----|----|----|----|-------|-------|------|
| cond  | 01    | I  | P  | U  | B  | W  | L  | Rn    | Rd    | Src2 |

The tables below show how I, P, U, B, W, & L contribute to the instruction.

| L | B | Instruction |
|---|---|-------------|
| 0 | 0 | STR         |
| 1 | 0 | LDR         |
| 1 | 1 | LDRB        |

| P | W | Index Mode |
|---|---|------------|
| 0 | 0 | Post-index |
| 1 | 0 | Offset     |
| 1 | 1 | Pre-index  |

| bit | I                        | U          |
|-----|--------------------------|------------|
| 0   | Immediate offset in Src2 | Subtract 1 |
| 1   | Register offset in Src2  | Add        |

### Branch Instruction

The branch instruction structure is much simpler as it only accounts for wheter it is a standard branch, or a branch with link. The branch with link is not implemented in this project, so "L" only has the option of 0.

| 31:28 | 27:26 | 25:24 | 23:0 |
|-------|-------|-------|------|
| cond  | 10    | 1L    | imm24|

| L      | Branch Type  |
|--------|--------------|
| Branch | Branch & Link|

### Condition Flags

The Current Program Status Register (CPSR) is responsible for holidng information on the condition flags. Thus, the structure will be as seen below,

| 31 | 30 | 29 | 28 | ...|
|----|----|----|----|----|
| N  | Z  | C  | V  | ...|

where N, Z, C, and V represent negative, zero, carry, and overflow, respectively.

The table below showcases the different types of mnemonics/conditions with flags.

| cond | mnemonic | name                                |
|------|----------|-------------------------------------|
| 0000 | EQ       | Equal                               |
| 0001 | NE       | NotEqual                            |
| 0010 | CS/HS    | Carry Set / Unsigned Higher or Same |
| 0011 | CC/LO    | Carry Clear / Unsigned Lower        |
| 0100 | MI       | Minus / Negative                    |
| 0101 | PL       | Plus / Positive or Zero             |
| 0110 | VS       | Overflow / Overflow Set             |
| 0111 | VC       | No Overflow / Overflow Clear        |
| 1000 | HI       | Unsigned Higher                     |
| 1001 | LS       | Unsigned Lower or Same              |
| 1010 | GE       | Signed Greater Than or Equal        |
| 1011 | LT       | Signed Less Than                    |
| 1100 | GT       | Signed Greater Than                 |
| 1101 | LE       | Signed Less Than or Equal           |
| 1110 | AL       | Always / Unconditional              |

## Schematics

The top level abstraction is illustrated below, where the Controller, Datapath, Instruction Memory, and Data Memory, when connected together, form the CPU. 

![top_level_abs](/schematics/top_level_design.svg)

The datapath, when connected to the data memory and controller, is illustrated below.

![datapath_datamem_controller](/schematics/microarchitecture.svg)

A closer look into the controller and the decoder within the controller.

![controller](/schematics/controller.svg)

![decoder](/schematics/decoder.svg)

For the controller, the following table is used to identify the operation of the instruction being passed.

| ALUOp | Funct 4:1 (cmd) | Funct][0] (S) | Type  | ALUControl 1:0 | FlagW 1:0 |
|-------|-----------------|--------------|-------|----------------|-----------|
| 0     | X               | X            | NOT DP| 00             | 00        |
| 1     | 100             | 0            | ADD   | 00             | 00        |
| 1     | 100             | 1            | ADD   | 00             | 11        |
| 1     | 010             | 0            | SUB   | 01             | 00        |
| 1     | 010             | 1            | SUB   | 01             | 11        |
| 1     | 000             | 0            | AND   | 10             | 00        |
| 1     | 000             | 1            | AND   | 10             | 10        |
| 1     | 1100            | 0            | ORR   | 11             | 00        |
| 1     | 1100            | 1            | ORR   | 11             | 10        |

// Insert Memory Module

## Verilog Implementation



## Verification of CPU



