#!/bin/bash

set -e
set -u

gcc -Wall -o test test.c -lwiringPi -I.
./test
