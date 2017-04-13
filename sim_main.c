/*
 * All Rights Reserved - 2016
 * Technion - Israel Institute of Technology, Faculty of Electrical Engineering.
 */
 
/* 046267 Computer Architecture - Spring 2016 - HW #1               */
/* Main program for simulation environment testing                  */
/* Usage: ./mysim <memory image filename> <number of cycle to run>  */

#include <stdlib.h>
#include <stdio.h>
#include "sim_api.h"

void DumpCoreState(SIM_coreState *state)
{
    int i;
    SIM_cmd *curCmd;
    char const *curCmdStr;

    struct
    {
        SIM_cmd cmd;      // The processed command in each pipe stage
        int32_t src1Val; // Actual value of src1 (considering forwarding mux, etc.)
        int32_t src2Val; // Actual value of src2 (considering forwarding mux, etc.)
    } pipeStageState[SIM_PIPELINE_DEPTH];

    printf("PC = 0x%X\n", state->pc);
    printf("Register file:\n");
    for (i = 0; i < SIM_REGFILE_SIZE; ++i)
        printf("\tR%d = 0x%X", i, state->regFile[i]);
    printf("\nCommand at each pipe stage:\n");
    for (i = 0; i < SIM_PIPELINE_DEPTH; ++i)
    {
        curCmd = &state->pipeStageState[i].cmd;
        if ((curCmd->opcode > CMD_MAX) || (curCmd->opcode < 0))
            curCmdStr = "<Invalid Cmd.>";
        else
            curCmdStr = cmdStr[curCmd->opcode];
        printf("\t%s : %s $%d , $%d(=0x%X) , %s%d(=0x%X)\n", pipeStageStr[i],
               curCmdStr, curCmd->dst, curCmd->src1,
               state->pipeStageState[i].src1Val,
               (curCmd->isSrc2Imm ? "" : "$"), curCmd->src2,
               state->pipeStageState[i].src2Val);
    }
}

int main(int argc, char const *argv[])
{
    int i, simDuration;
    ;
    char const *memFname = argv[1];
    char const *simDurationStr = argv[2];

    SIM_coreState curState;

    if (argc != 3)
    {
        fprintf(stderr,
                "Usage: %s <memory image filename> <number of cycles to run>\n",
                argv[0]);
        exit(1);
    }

    /* Initialized simulation modules */
    printf("Loading memory image file: %s\n", memFname);
    if (SIM_MemReset(memFname) != 0)
    {
        fprintf(stderr, "Failed initializing memory simulator!\n");
        exit(2);
    }

    printf("Reseting core...\n");
    if (SIM_CoreReset() != 0)
    {
        fprintf(stderr, "Failed reseting core!\n");
        exit(3);
    }
    /* Running simulation */
    simDuration = atoi(simDurationStr);
    if (simDuration <= 0)
    {
        fprintf(stderr, "Invalid simulation durection argument: %s\n",
                simDurationStr);
        exit(4);
    }
    printf("Running simulation for %d cycles", simDuration);
    for (i = 0; i < simDuration; ++i)
    {
        SIM_CoreClkTick();
        SIM_MemClkTick();
    }

    printf("Simulation finished. Final state is:\n");
    SIM_CoreGetState(&curState);
    DumpCoreState(&curState);

    return 0;
}
