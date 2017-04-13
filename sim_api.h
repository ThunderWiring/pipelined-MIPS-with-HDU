/*
 * All Rights Reserved - 2016
 * Technion - Israel Institute of Technology, Faculty of Electrical Engineering.
 */
  
/* 046267 Computer Architecture - Spring 2016 - HW #1 */
/* API for the in-order pipelined processor simulator */

#ifndef _SIM_API_H_
#define _SIM_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define SIM_PIPELINE_DEPTH 5  /* Number of pipeline stages */
#define SIM_REGFILE_SIZE 32   /* Number of (general purpose) registers in the register file (all integer) */

/*! Commands opcodes */
typedef enum
{
    CMD_NOP = 0,
    CMD_ADD,    // dst <- src1 + src2
    CMD_SUB,    // dst <- src1 - src2
    CMD_LOAD,   // dst <- Mem[src1 + src2]  (src2 may be an immediate)
    CMD_STORE,  // Mem[dst + src2] <- src1  (src2 may be an immediate)
    CMD_BR,     // Unconditional relative branch to PC+dst register value
    CMD_BREQ,   // Branch to PC+dst if (src1 == src2)
    CMD_BRNEQ,  // Branch to PC+dst if (src1 != src2)
    CMD_MAX = CMD_BRNEQ
} SIM_cmd_opcode;

typedef struct
{
    SIM_cmd_opcode opcode;
    int src1;    // Source 1: Index of register
    int32_t src2;    // Source 2: Index of register OR immediate (see 'isSrc2Imm')
    bool isSrc2Imm; // Is src2 an immediate value and not a register index
    int dst;     // Destination: Index of register
} SIM_cmd;


/*! A structure to return information about the currect simulator state */
typedef struct
{
    int32_t pc; /// Value of the current program counter (at instruction fetch stage)
    int32_t regFile[SIM_REGFILE_SIZE]; /// Values of each register in the register file
    struct
    {
        SIM_cmd cmd;      /// The processed command in each pipe stage
        int32_t src1Val;  /// Actual value of src1 (considering forwarding mux, etc.)
        int32_t src2Val;  /// Actual value of src2 (considering forwarding mux, etc.)
    } pipeStageState[SIM_PIPELINE_DEPTH];
} SIM_coreState;

/*! Lookup table from command enumeration to command name */
static const char *cmdStr[] = { "NOP", "ADD", "SUB", "LOAD", "STORE", "BR", "BREQ", "BRNEQ" };

/*! Lookup table from pipe stage to its name - useful for debugging */
static const char *pipeStageStr[] = { "IF", "ID", "EXE", "MEM", "WB" };



/*************************************************************************/
/* The memory simulator API - implemented in sim_mem.c                   */
/*************************************************************************/

/*! SIM_MemReset: Reset the memory simulator and load memory image
  \param[in] memImgFname Memory image filename
  The memory image filename is composed from segments of 2 types, defined by an "@" location/type line:
  1. "I@<address>" : The following lines are instructions at given memory offset.
     Each subsequent line up to the next "@" line is an instruction of format: <command> <dst>,<src1>,<src2>
     Commands is one of: NOP, ADD, SUB, LOAD, STORE, BR, BREQ, BRNEQ
     operands are $<num> for any general purpose register, or just a number for immediate (for src2 only)
  2. "D@<address>" : The following lines are data values at given memory offset.
     Each subsequent line up the the next "@"is data value of a 32 bit (hex.) data word, e.g., 0x12A556FF
  \returns 0 - for success in reseting and loading image file. <0 in case of error.

  * Any memory address that is not defined in the given image file is initialized to zero.
 */
int SIM_MemReset(const char *memImgFname);

/*! SIM_MemClkTick(): Update the memory simulator state given a single clock tick.
  (we assume the memory model is synchronous)
*/
void SIM_MemClkTick();

/*! SIM_ReadDataMem: Read data from main memory simulator
  \param[in] addr The memory location to read.
                  Note that while we read 32 bit data words, addressing is per byte, i.e., the address must be aligned to 4.
  \param[out] dst The destination location to read into
  \returns 0 - for success in reading. (-1) for memory wait-state
  * In case of a wait-state error code, one should invoke again on the next clock cycle.
*/
int SIM_MemDataRead(uint32_t addr, int32_t *dst);

/*! SIM_MemDataWrite: Write a value to given memory address
  \param[in] addr The main memory address to write. Must be 4-byte-aligned
  \param[in] val  The value to write
*/
void SIM_MemDataWrite(uint32_t addr, int32_t val);

// TODO - add description
/*! SIM_ReadInstMem: Read instruction from main memory simulator
  \param[in] addr The memory location to read.
                  Note that while we read 32 bit data words, addressing is per byte, i.e., the address must be aligned to 4.
  \param[out] dst The destination location to read into
*/
void SIM_MemInstRead(uint32_t addr, SIM_cmd *dst);

/*************************************************************************/
/* The following functions should be implemented in your sim.c (or .cpp) */
/*************************************************************************/

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - All pipeline stages are empty.
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void);

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick(void);

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
  \param[out] curState The returned current pipeline state
*/
void SIM_CoreGetState(SIM_coreState *curState);



#ifdef __cplusplus
}
#endif

#endif /*_SIM_API_H_*/
