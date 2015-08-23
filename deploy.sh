#!/bin/bash

set -e
set -u

source deploy.settings

rsync -rtv \
	--delete \
	--itemize-changes \
	--exclude '*.pyc' \
	--exclude 'override.ini' \
	*.c \
	*.cpp \
	*.h \
	*.sh \
	Makefile \
	pi@$PI_HOSTNAME:test_acc/
