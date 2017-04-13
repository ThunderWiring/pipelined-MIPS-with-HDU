/*
 * All Rights Reserved - 2016
 * Technion - Israel Institute of Technology, Faculty of Electrical Engineering.
 */
 
/* 046267 Computer Architecture - Spring 2016 - HW #1 */
/* Main memory simulator implementation               */

#include "sim_api.h"

uint32_t prog_start; // the addr of the code block
uint32_t data_start; // the addr of the data block
SIM_cmd instructions[100]; // where the instructions are kept
int32_t data[100]; // where the data is kept
uint32_t ticks; // the current clk tick
uint32_t read_tick; // the clk tick of the first attempt to read

typedef struct
{
    uint32_t addr;
    int32_t val;
    bool valid;
    uint32_t ticks; // for LRU
} cache_line;

cache_line cache[8];

uint32_t get_start(char *line)
{
    line = strtok(line, "\n");
    strtok(line, "@");
    line = strtok(NULL, "@");
    return (uint32_t) strtol(line, NULL, 0);
}

void get_data(char* line, int data_i)
{
    line = strtok(line, "\n");
    data[data_i] = (int32_t) strtol(line, NULL, 0);
}

int get_dst(char *dst)
{
    strtok(dst, ",");
    strtok(dst, "$");
    dst = strtok(NULL, "$");
    return atoi(dst);
}

int get_dst_br(char *dst)
{
    strtok(dst, "\n");
    strtok(dst, "$");
    dst = strtok(NULL, "$");
    return atoi(dst);
}

int get_src1(char *src1)
{
    strtok(src1, ",");
    src1 = strtok(NULL, ",");
    strtok(src1, "$");
    src1 = strtok(NULL, "$");
    return atoi(src1);
}

int get_src2(char *src2)
{
    strtok(src2, ",");
    strtok(NULL, ",");
    src2 = strtok(NULL, ",");
    strtok(src2, "$");
    src2 = strtok(NULL, "$");
    src2 = strtok(src2, "\n");
    return atoi(src2);
}

int get_src2_imm(char *src2, int inst_num)
{
    strtok(src2, ",");
    strtok(NULL, ",");
    src2 = strtok(NULL, ",");
    if (strchr(src2, '$') == NULL)
    {
        strtok(src2, " ");
        instructions[inst_num].isSrc2Imm = 1;
    }
    else
    {
        strtok(src2, "$");
        src2 = strtok(NULL, "$");
        //assert(instructions[inst_num].isSrc2Imm == 0);
    }
    src2 = strtok(src2, "\n");
    if (strchr(src2, 'x') == NULL)
    {
        return atoi(src2);
    }
    else
    {
        return (uint32_t) strtol(src2, NULL, 0);
    }
}

void add_sub_branch(char *line, int inst_num)
{
    char dst[50];
    memset(dst, '\0', sizeof(dst));
    strcpy(dst, line);
    instructions[inst_num].dst = get_dst(dst);
    char src1[50];
    memset(src1, '\0', sizeof(src1));
    strcpy(src1, line);
    instructions[inst_num].src1 = get_src1(src1);
    char src2[50];
    memset(src2, '\0', sizeof(src2));
    strcpy(src2, line);
    instructions[inst_num].src2 = get_src2_imm(src2, inst_num);
}

void load_store(char* line, int inst_num)
{
    char dst[50];
    memset(dst, '\0', sizeof(dst));
    strcpy(dst, line);
    instructions[inst_num].dst = get_dst(dst);
    char src1[50];
    memset(src1, '\0', sizeof(src1));
    strcpy(src1, line);
    instructions[inst_num].src1 = get_src1(src1);
    char src2[50];
    memset(src2, '\0', sizeof(src2));
    strcpy(src2, line);
    instructions[inst_num].src2 = get_src2_imm(src2, inst_num);
}

void branch(char* line, int inst_num)
{
    char dst[50];
    memset(dst, '\0', sizeof(dst));
    strcpy(dst, line);
    instructions[inst_num].dst = get_dst_br(dst);
}

void get_inst(char *line, int inst_num)
{
    char command[50];
    memset(command, '\0', sizeof(command));
    strcpy(command, line);
    strtok(command, " ");
    int opc = 0;
    while (strcmp(command, cmdStr[opc]) != 0)
    {
        ++opc;
    }
    instructions[inst_num].opcode = opc;
    switch (opc)
    {
    case 0: // NOP
        break;
    case 1:
    case 2:
        add_sub_branch(line, inst_num);
        break;
    case 3:
    case 4:
        load_store(line, inst_num);
        break;
    case 5:
        branch(line, inst_num);
        break;
    case 6:
    case 7:
        add_sub_branch(line, inst_num);
        break;
    }
}

int SIM_MemReset(const char *memImgFname)
{
    FILE *img = fopen(memImgFname, "r");
    char line[1024];
    if (img == 0)
    {
        return -1; // can't open img file
    }
    while (fgets(line, 1024, img) != NULL)
    {
        if (line[0] == '#' || line[0] == '\n')   // comment or empty line
        {
            continue;
        }
        else if (line[0] == 'I' && line[1] == '@')     // start of code block
        {
            prog_start = get_start(line);
            int inst = 0;
            fgets(line, 1024, img);
            // get next instructions
            while (line[0] != '\n' && line[0] != '#' && line[0] != 'D')
            {
                get_inst(line, inst);
                ++inst;
                if (fgets(line, 1024, img) == NULL)   //EOF
                {
                    break;
                }
            }
        }
        else if (line[0] == 'D' && line[1] == '@')     // start of data block
        {
            data_start = get_start(line);
            int data_i = 0;
            fgets(line, 1024, img);
            while (line[0] != '\n' && line[0] != '#' && line[0] != 'I')
            {
                get_data(line, data_i);
                ++data_i;
                if (fgets(line, 1024, img) == NULL)
                {
                    break;
                }
            }
        }
    }
    fclose(img);
    return 0;
}

void SIM_MemClkTick()
{
    ++ticks;
}

int cache_lookup(uint32_t addr)
{
    int i;
    for (i = 0; i < 8; ++i)
    {
        if (cache[i].addr == addr)
        {
            return i;
        }
    }
    return -1;
}

void insert_to_cache(uint32_t addr)
{
    int i;
    int addr_i = addr - data_start;
    addr_i = addr_i / 4;
    // insert if there is an empty space
    for (i = 0; i < 8; ++i)
    {
        if (cache[i].valid == 0)
        {
            cache[i].addr = addr;
            cache[i].val = data[addr_i];
            cache[i].valid = 1;
            cache[i].ticks = ticks;
            return;
        }
    }
    // no empty space, find LRU
    int remove = -1;
    int max_ticks = 0;
    for (i = 0; i < 8; ++i)
    {
        if ((ticks - cache[i].ticks) > max_ticks)
        {
            max_ticks = (ticks - cache[i].ticks);
            remove = i;
        }
    }
    // insert instead of LRU
    cache[remove].addr = addr;
    cache[remove].val = data[addr_i];
    cache[remove].ticks = ticks;
    cache[remove].valid = 1;
}

int SIM_MemDataRead(uint32_t addr, int32_t *dst)
{
    // init read tick
    if (read_tick == 0)
    {
        read_tick = ticks;
    }
    // calc location in data array
    int addr_i = addr - data_start;
    addr_i = addr_i / 4;
    // first attempt to read
    if (read_tick == ticks)
    {
        // find in cache
        int i = cache_lookup(addr);
        if (i != -1)
        {
            //assert(cache[i].val == data[addr_i]);
            *dst = cache[i].val;
            cache[i].ticks = ticks;
            //assert(cache[i].valid == 1);
            read_tick = 0; // init for next read
            return 0;
        }
        else
        {
            insert_to_cache(addr);
        }
    }
    if ((ticks - read_tick) < 3)
    {
        return -1;
    }
    *dst = data[addr_i];
    read_tick = 0; // init for next read
    return 0;
}

void SIM_MemDataWrite(uint32_t addr, int32_t val)
{
    int addr_i = addr - data_start;
    addr_i = addr_i / 4; // addr is aligned to 4 byte
    data[addr_i] = val;
    int i = cache_lookup(addr);
    // if it is in cache then update the cache
    if (i != -1)
    {
        cache[i].val = val;
        cache[i].ticks = ticks; // for LRU
        //assert(cache[i].valid == 1);
    }
}

void SIM_MemInstRead(uint32_t addr, SIM_cmd *dst)
{
    addr = addr - prog_start;
    addr = addr / 4; // addr is aligned to 4 byte
    dst->opcode = instructions[addr].opcode;
    dst->dst = instructions[addr].dst;
    dst->src1 = instructions[addr].src1;
    dst->src2 = instructions[addr].src2;
    dst->isSrc2Imm = instructions[addr].isSrc2Imm;
}
