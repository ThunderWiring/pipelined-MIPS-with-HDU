/*
* Copyright (c) 2016 Bassam Yassin
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


/* 046267 Computer Architecture - Spring 2016 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

/*
 * Author: Bassam Yassin
 *
 * 	 5 April 2016
 */

#include "sim_api.h"

#define RESET_SUCCESS 		 0
#define RESET_FAIL			-1
#define REGISTER_RESET_VALUE 0
#define PC_RESET_VALUE		 0
#define IF  0
#define ID  1
#define EXE 2
#define MEM 3
#define WB  4
#define DATA_HAZARD_CORRECTED   0
#define DATA_HAZARD_INIVETABLE -1
#define MEMORY_READ_SUCCESS     0
#define MEMORY_READ_WAIT_STATE -1
#define PC_INCREMENT 			4

typedef uint32_t pc_t;

/*
 *
 * [] cmd : The processed command in each pipe stage.
 * [] src1Val : Actual value of src1 (considering forwarding mux, etc.)
 * [] src2Val : Actual value of src2 (considering forwarding mux, etc.)
 * [] dstVal  : Actual Value of dst register after EXE stage.
 * 			 this value either gets written to the regFile (in WB stage)
 * 			  or to Data Memory (in MEM stage).
 * */
typedef struct pipeStageState{
	SIM_cmd cmd;
	pc_t PCofCmd;
	int32_t src1Val;
	int32_t src2Val;
	int32_t dstVal;
} PipeState;
/*
 * Data structure of the pipeline stages' state (snapshot of the core).
 * [] pc : Value of the current program counter (at instruction fetch stage).
 * [] regFile : Array of values of each register in the register file.
 * [] memoryReadWait: flag indicating whether the reading from the memory didn't end
 * 					in one cycle, and should try to read in the next cycle.
 * [] branchTaken: 'true' iff last branch cmd is taken. 'false' otherwise.
 * [] stall: a flag that indicates whether a hazard had been detected -
 * 			'true' if we need to stall (put NOP in between cmd in ID and EXE,
 * 			s.t. in the next cycle, the cmd in ID remains there and the NOP proceeds to EXE).
 * [] branch_dst: the address to assign to PC iff branchTaken = true.

 * */
typedef struct state_t {
	int32_t pc;
	int32_t regFile[SIM_REGFILE_SIZE];
	bool afterReset;
	bool memoryReadWait;
	bool branchTaken;
	bool stall;
	int32_t branch_dst;
	PipeState pipeStageState[SIM_PIPELINE_DEPTH];
} State;

static State currentState; /* Global state of the Core. */
static PipeState lastIterationInstructions[SIM_PIPELINE_DEPTH];

/******************************************************************************/
// Static functions (auxiliary)
/******************************************************************************/
/*
 * assigns NOP cmd in the specified stage of the pipeline.
 * */
static void flush(int stage) {
	if(stage < IF || stage > WB) {
		return;
	}
	currentState.pipeStageState[stage].dstVal = 0;
	currentState.pipeStageState[stage].src1Val = 0;
	currentState.pipeStageState[stage].src2Val = 0;
	currentState.pipeStageState[stage].cmd.dst = 0;
	currentState.pipeStageState[stage].cmd.isSrc2Imm = false;
	currentState.pipeStageState[stage].cmd.opcode = CMD_NOP;
	currentState.pipeStageState[stage].cmd.src1 = 0;
	currentState.pipeStageState[stage].cmd.src2 = 0;
}

/*
 * copies the struct of SIM_cmd pointer between the 2 input params.
 * */
static void copyPipeState(PipeState* to, PipeState* from) {
	if(!to || !from) {
		return;
	}
	to->PCofCmd = from->PCofCmd;
	to->cmd.dst = from->cmd.dst;
	to->cmd.isSrc2Imm = from->cmd.isSrc2Imm;
	to->cmd.opcode = from->cmd.opcode;
	to->cmd.src1 = from->cmd.src1;
	to->cmd.src2 = from->cmd.src2;
	to->dstVal = from->dstVal;
	to->src1Val = from->src1Val;
	to->src2Val = from->src2Val;
}

/*
 * returns the pc of the next command.
 * */
static pc_t updatePC() {
	if(currentState.branchTaken) {
		flush(IF);
		flush(ID);
		flush(EXE);
		copyPipeState(lastIterationInstructions + IF, &currentState.pipeStageState[IF]);
		copyPipeState(lastIterationInstructions + ID, &currentState.pipeStageState[ID]);
		copyPipeState(lastIterationInstructions + EXE, &currentState.pipeStageState[EXE]);
		currentState.branchTaken = false;
		currentState.pc = currentState.branch_dst;
	} else if(currentState.afterReset) {
		currentState.pc = PC_RESET_VALUE;
		currentState.afterReset = false;
	} else {
		currentState.pc += PC_INCREMENT;
	}
	return currentState.pc;
}

/*
 * Detect data hazards which cannot be solved by the forwarding units.
 * such hazards stems from 2 consecutive instructions with true dependency between them.
 * */
static bool detectHazard() {
	SIM_cmd_opcode _opcode = currentState.pipeStageState[EXE].cmd.opcode;
	int _dst_index = currentState.pipeStageState[EXE].cmd.dst;
	int _src1 = currentState.pipeStageState[ID].cmd.src1;
	int _src2 = currentState.pipeStageState[ID].cmd.isSrc2Imm ?
			-1 : currentState.pipeStageState[ID].cmd.src2;
	if(_opcode == CMD_LOAD && (_dst_index == _src1 || _dst_index == _src2)) {
		currentState.stall = true;
		return true;
	}
	return false;
}

/*
 * detects data hazards and forwards updated registers' values when necessary,
 * from WB, MEM to EXE.
 * -----------
 * forwarding from MEM to EXE:
 * 		when src1 or src2 registers in EXE stage was dst register in MEM stage.
 *  forwarding from WB to EXE:
 * 		when src1 or src2 registers in EXE stage was dst register in WB stage.
 *
 * In case the command in WB and/or MEM are *NOT* one of: ADD, SUB, LOAD,
 * then there is nothing to forward!
 * */
static void forwardUnit() {
	PipeState MEM_dummy, WB_dummy;
	copyPipeState(&MEM_dummy, lastIterationInstructions + EXE);
	copyPipeState(&WB_dummy, lastIterationInstructions + MEM);
	int mem_dst_index = MEM_dummy.cmd.dst;
	int WB_dst_index = WB_dummy.cmd.dst;
	int exe_src1_index = currentState.pipeStageState[EXE].cmd.src1;
	int exe_src2_index = currentState.pipeStageState[EXE].cmd.src2;
	int exe_dst_index = currentState.pipeStageState[EXE].cmd.dst;
	bool notImm = !currentState.pipeStageState[EXE].cmd.isSrc2Imm;
	SIM_cmd_opcode EXE_opcode = currentState.pipeStageState[EXE].cmd.opcode;
	SIM_cmd_opcode MEM_opcode = MEM_dummy.cmd.opcode;
	SIM_cmd_opcode WB_opcode = WB_dummy.cmd.opcode;
	bool WB_validFWD = (WB_opcode == CMD_ADD) || (WB_opcode == CMD_SUB) || (WB_opcode == CMD_LOAD)  ;
	bool MEM_validFWD = (MEM_opcode  == CMD_ADD) || (MEM_opcode  == CMD_SUB) || (MEM_opcode  == CMD_LOAD)  ;
	if(WB_validFWD) {
		if(WB_dst_index == exe_src1_index) {
			currentState.pipeStageState[EXE].src1Val = WB_dummy.dstVal;
		}
		if(notImm && (WB_dst_index == exe_src2_index)) {
			currentState.pipeStageState[EXE].src2Val = WB_dummy.dstVal;
		}
		if((EXE_opcode == CMD_BREQ || EXE_opcode == CMD_BRNEQ || EXE_opcode == CMD_BR
				|| EXE_opcode == CMD_STORE) && (exe_dst_index == WB_dst_index)) {
			currentState.pipeStageState[EXE].dstVal = WB_dummy.dstVal;
		}
	}
	if(MEM_validFWD) {
		if(mem_dst_index == exe_src1_index) {
			currentState.pipeStageState[EXE].src1Val = MEM_dummy.dstVal;
		}
		if (notImm && (mem_dst_index == exe_src2_index)) {
			currentState.pipeStageState[EXE].src2Val = MEM_dummy.dstVal;
		}
		if((EXE_opcode == CMD_BREQ || EXE_opcode == CMD_BRNEQ || EXE_opcode == CMD_BR
				|| EXE_opcode == CMD_STORE) && (exe_dst_index == mem_dst_index)) {
			currentState.pipeStageState[EXE].dstVal = MEM_dummy.dstVal;
		}
	}
}

/*
 * fetch the next instruction from the instructions' memory to the IF stage.
 * */
static void fetch() {
	SIM_cmd newInstruction;
	pc_t pc = updatePC();
	SIM_MemInstRead(pc, &newInstruction);
	currentState.pipeStageState[IF].PCofCmd = pc;
	currentState.pipeStageState[IF].dstVal = REGISTER_RESET_VALUE;
	currentState.pipeStageState[IF].src1Val = REGISTER_RESET_VALUE;
	currentState.pipeStageState[IF].src2Val = REGISTER_RESET_VALUE;
	currentState.pipeStageState[IF].cmd.dst = newInstruction.dst;
	currentState.pipeStageState[IF].cmd.isSrc2Imm = newInstruction.isSrc2Imm;
	currentState.pipeStageState[IF].cmd.opcode = newInstruction.opcode;
	currentState.pipeStageState[IF].cmd.src1 = newInstruction.src1;
	currentState.pipeStageState[IF].cmd.src2 = newInstruction.src2;
}

/*
 * [1] Replace the instruction in the ID stage with the new instruction (new_cmd).
 * [2] Update the current state accordingly:
 * 		 (*) change the SIM_cmd field in the pipeStageState[ID], as well as the
 * 		 actual values of the registers.
 * 		 (*) does NOT change in the register file!
 * 	-----------------
 * @input - new_cmd: new instruction in the IF/ID pipe which to be decoded next.
 *  This is the instruction that was in the IF pipe at the previous cycle.
 * */
static void decode(PipeState* new_cmd) {
	copyPipeState(&currentState.pipeStageState[ID], new_cmd);
	switch (new_cmd->cmd.opcode) {
	case CMD_NOP:
	case CMD_BR:
		currentState.pipeStageState[ID].src1Val = currentState.regFile[new_cmd->cmd.src1];
		currentState.pipeStageState[ID].src2Val = currentState.regFile[new_cmd->cmd.src2];
		currentState.pipeStageState[ID].dstVal = currentState.regFile[new_cmd->cmd.dst];
		break;
	case CMD_ADD:
	case CMD_SUB:
	case CMD_LOAD:
	case CMD_STORE:
	case CMD_BREQ:
	case CMD_BRNEQ:
		currentState.pipeStageState[ID].src1Val = currentState.regFile[new_cmd->cmd.src1];
		currentState.pipeStageState[ID].src2Val = new_cmd->cmd.isSrc2Imm ?
							new_cmd->cmd.src2 : currentState.regFile[new_cmd->cmd.src2];
		currentState.pipeStageState[ID].dstVal = currentState.regFile[new_cmd->cmd.dst];
		break;
	default: //should never reach!
		printf("Error - [%s]: unrecognized command!\n", __FUNCTION__);
		return;
	}
}

/*
 * [1] Replace the instruction in the ID stage with the new instruction (new_cmd).
 * [2] check for data hazards:
 * 		(*) in such case, get the updated values from MEM or WB stages.
 * [3] for branch commands BREQ, BRNEQ: in this stage of the pipeline (EXE),
 * the dst register holds the new pc to be jumped to in case the branch is taken
 * [4] LOAD: Register src1 holds the value of the place in memory to read from.
 * */
static void execute(PipeState* new_cmd) {
	copyPipeState(&currentState.pipeStageState[EXE], new_cmd);
	detectHazard();
	forwardUnit();
	switch (new_cmd->cmd.opcode) {
	case CMD_NOP:
		break;
	case CMD_SUB:
		currentState.pipeStageState[EXE].dstVal =
						currentState.pipeStageState[EXE].src1Val
								- currentState.pipeStageState[EXE].src2Val;
		break;
	case CMD_ADD:
		currentState.pipeStageState[EXE].dstVal =
				currentState.pipeStageState[EXE].src1Val +
							currentState.pipeStageState[EXE].src2Val;
		break;
	case CMD_LOAD:
		currentState.pipeStageState[EXE].dstVal = currentState.pipeStageState[EXE].src1Val +
									currentState.pipeStageState[EXE].src2Val;
		break;
	case CMD_STORE:
	case CMD_BR:
	case CMD_BREQ:
	case CMD_BRNEQ:
		break;
	default: //should never reach!
		printf("Error - [%s]: unrecognized command!\n", __FUNCTION__);
		return;
	}
}
/*
 * [] branch resolution - calculate the new PC.
 * [] lw: read the data from memory into the register.
 * [] sw: store the data into the calculated place in memory.
 * */
static void memory(PipeState* new_cmd) {
	copyPipeState(&currentState.pipeStageState[MEM], new_cmd);
	int32_t writeAddress;
	switch (new_cmd->cmd.opcode) {
	case CMD_NOP:
	case CMD_ADD:
	case CMD_SUB:
		break;
	case CMD_BR:
		currentState.branchTaken = true;
		currentState.branch_dst = currentState.pipeStageState[MEM].dstVal +
				currentState.pipeStageState[MEM].PCofCmd + PC_INCREMENT;
		break;
	case CMD_BREQ:
		if(currentState.pipeStageState[MEM].src1Val ==
				 currentState.pipeStageState[MEM].src2Val) {
			currentState.branchTaken = true;
			currentState.branch_dst = currentState.pipeStageState[MEM].dstVal +
					currentState.pipeStageState[MEM].PCofCmd + PC_INCREMENT;
		}
		break;
	case CMD_BRNEQ:
		if(currentState.pipeStageState[MEM].src1Val !=
				 currentState.pipeStageState[MEM].src2Val) {
			currentState.branchTaken = true;
			currentState.branch_dst = currentState.pipeStageState[MEM].dstVal +
					currentState.pipeStageState[MEM].PCofCmd + PC_INCREMENT;
		}
		break;
	case CMD_LOAD:
		if(SIM_MemDataRead(currentState.pipeStageState[MEM].dstVal,
				&currentState.pipeStageState[MEM].dstVal) == MEMORY_READ_WAIT_STATE) {
			currentState.memoryReadWait = true;
		}
		break;
	case CMD_STORE:
		writeAddress = currentState.pipeStageState[MEM].dstVal +
									currentState.pipeStageState[MEM].src2Val;
		SIM_MemDataWrite(writeAddress, currentState.pipeStageState[MEM].src1Val);
		break;
	default: //should never reach!
		printf("Error - [%s]: unrecognized command!\n", __FUNCTION__);
		return;
	}
}

/*
 * [] updates register file
 * [] updates src1Val, src2Val and dstVal from early ID stage.
 * */
static void writeBack(PipeState* new_cmd) {
	copyPipeState(&currentState.pipeStageState[WB], new_cmd);
	int dst_index = new_cmd->cmd.dst;
	int32_t dst_val = new_cmd->dstVal;
	switch (new_cmd->cmd.opcode) {
	case CMD_ADD:
	case CMD_SUB:
	case CMD_LOAD:
		currentState.regFile[dst_index] = dst_val;
		break;
	case CMD_NOP:
	case CMD_STORE:
	case CMD_BR:
	case CMD_BREQ:
	case CMD_BRNEQ:
		break;
	default: //should never reach!
		printf("Error - [%s]: unrecognized command!\n", __FUNCTION__);
		return;
	}
	//update src1,2 and dst value from ID stage:
	currentState.pipeStageState[ID].dstVal = currentState.regFile[currentState.pipeStageState[ID].cmd.dst];
	currentState.pipeStageState[ID].src1Val = currentState.regFile[currentState.pipeStageState[ID].cmd.src1];
	if(!currentState.pipeStageState[ID].cmd.isSrc2Imm) {
		currentState.pipeStageState[ID].src2Val = currentState.regFile[currentState.pipeStageState[ID].cmd.src2];
	}
}
/******************************************************************************/
static void resetPipeState(PipeState* pipe) {
	pipe->PCofCmd = PC_RESET_VALUE;

	pipe->cmd.dst = 0;
	pipe->cmd.isSrc2Imm = false;
	pipe->cmd.opcode = CMD_NOP;
	pipe->cmd.src1 = 0;
	pipe->cmd.src2 = 0;

	pipe->src1Val = REGISTER_RESET_VALUE;
	pipe->src2Val = REGISTER_RESET_VALUE;
	pipe->dstVal = REGISTER_RESET_VALUE;
}
/*
 * - reset regFile.
 * - put NOPs in all pipes, and loads cmd at 0x0 to IF stage.
 * - assign 0x0 to pc.
 * */
int SIM_CoreReset(void) {
	flush(IF);
	flush(ID);
	flush(EXE);
	flush(MEM);
	flush(WB);
	currentState.pc = PC_RESET_VALUE;
	currentState.afterReset = true;
	currentState.branchTaken = false;
	currentState.branch_dst = 0;
	currentState.memoryReadWait = false;
	currentState.stall = false;
	for (int i = 0; i < SIM_REGFILE_SIZE; i++) {
		currentState.regFile[i] = REGISTER_RESET_VALUE;
	}
	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++) {
		resetPipeState(currentState.pipeStageState + i);
		resetPipeState(lastIterationInstructions + i);
	}
	fetch();
	currentState.afterReset = true;
	return RESET_SUCCESS;
}

/******************************************************************************/
/*
 * Implementing the 5 levels of the pipeline: IF, ID, EX, MEM, WB.
 * */
void SIM_CoreClkTick(void) {
	if(currentState.memoryReadWait) {
		currentState.memoryReadWait = false;
		writeBack(lastIterationInstructions + MEM); //write-back should continue as normal.
		memory(lastIterationInstructions + EXE);
		flush(WB);
		return;
	}
	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++) {
		copyPipeState(lastIterationInstructions + i, &currentState.pipeStageState[i]);
	}
	/*
	 * Note that it is meaningless to stall if branch is taken.
	 * That's because stalling separates between cmd in ID and EXE by-
	 * putting NOP in EXE, advancing EXE cmd to MEM pipe and stall cmd in ID.
	 * However, if branch is taken, then we'll flush IF, ID and EXE regardless!
	 * thus, no need to worry about stall-hazards in this specific case.
	 * */
	if(currentState.stall && !currentState.branchTaken) {
		flush(EXE);
		currentState.stall = false;
	}
	else {
		if(currentState.stall && currentState.branchTaken) {
			currentState.stall = false;
		}
		if(currentState.afterReset) {
			currentState.afterReset = false;
		} else {
			fetch();
			decode(lastIterationInstructions + IF);
			execute(lastIterationInstructions + ID);
		}
	}
	memory(lastIterationInstructions + EXE);
	writeBack(lastIterationInstructions + MEM);
}
/******************************************************************************/
void SIM_CoreGetState(SIM_coreState *curState) {
	curState->pc = currentState.pc;
	for(int i = 0; i < SIM_PIPELINE_DEPTH; i++) {
		curState->pipeStageState[i].src1Val = currentState.pipeStageState[i].src1Val;
		curState->pipeStageState[i].src2Val = currentState.pipeStageState[i].src2Val;
		curState->pipeStageState[i].cmd.dst = currentState.pipeStageState[i].cmd.dst;
		curState->pipeStageState[i].cmd.isSrc2Imm = currentState.pipeStageState[i].cmd.isSrc2Imm;
		curState->pipeStageState[i].cmd.opcode = currentState.pipeStageState[i].cmd.opcode;
		curState->pipeStageState[i].cmd.src1 = currentState.pipeStageState[i].cmd.src1;
		curState->pipeStageState[i].cmd.src2 = currentState.pipeStageState[i].cmd.src2;
	}
	for(int j = 0; j < SIM_REGFILE_SIZE; j++) {
		curState->regFile[j] = currentState.regFile[j];
	}
}
