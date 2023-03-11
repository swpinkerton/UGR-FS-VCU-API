#include "fs-ai_api.cpp"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


int a = 1;
int b = 2;
int c = fs_ai_api_test(a,b);



int main(){
    printf("out: %d",c);
    return 0;
}