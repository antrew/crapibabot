#!/bin/bash

set -e
set -u

source deploy.settings

rsync -rtv \
	--delete \
	--exclude '*.pyc' \
	--exclude 'override.ini' \
	*.cpp \
	*.h \
	*.sh \
	Makefile \
	pi@$PI_HOSTNAME:test_acc/
