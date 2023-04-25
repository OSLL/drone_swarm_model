#! /bin/bash

git clone https://github.com/openedx/xqueue-watcher.git
cd xqueue-watcher
make requirements
ln -s ../grader grader
cd ..