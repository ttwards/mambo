/**
 * @file main.c
 * write your description here
 */

#include <stdio.h>

int add(int a, int b);

int main() {

    /* initialize variables */
    int num1 = 5;                   // the first number
    int num2 = 10;                  // the second number
    int result = add(num1, num2);   //the result of adding num1 and num2

    /* print the result */
    printf("The sum of %d and %d is %d\n", num1, num2, result);
    return 0;
}

/**
 * @brief Adds two integers.
 *
 * @param a The first integer.
 * @param b The second integer.
 * @return The sum of a and b.
 */
int add(int a, int b) {
    return a + b;
}