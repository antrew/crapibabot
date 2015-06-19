#!/bin/bash

set -e
set -u

source deploy.settings

rsync -rtv \
	--delete \
	--exclude '*.pyc' \
	--exclude 'override.ini' \
	*.c \
	*.h \
	pi@$PI_HOSTNAME:test_c/
