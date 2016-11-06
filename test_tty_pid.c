#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/wait.h>
#include<sys/stat.h>
#include<fcntl.h>
int main(){
  int status,i;
  int fd;
  pid_t pid;
  char *p;
  if((p=getenv("PATH"))){
    printf("USER=%s\n",p);
  }

  char* file = "/dev/ttyUSB0";
  fd = open(file, O_RDONLY);
  printf("%s",file);
  if(isatty(fd)){
    printf(" is a tty.\n");
    printf("ttyname = %s\n",ttyname(fd));
  }
  else{
    printf("is not a tty.\n");
  }


  printf("pid = %d\n", getpid());
  printf("pid = %d\n", getppid());
  if(fork() ==0){
    printf("This is the child process.pid=%d\n",getpid());
    exit(7);
  }
  else{
    sleep(1);
    printf("This is the parent process,wait for child...\n");
    pid = wait(&status);
    i = WEXITSTATUS(status);
    printf("child's pid = %d.exit status = %d\n",pid,i);
  }

  close(fd);
  return 0;
}
