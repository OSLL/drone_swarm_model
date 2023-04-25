#! /bin/bash

if [ ! -d "xqueue_watcher" ]; then
	/bin/bash setup.sh
fi

cd xqueue-watcher
python3 -m xqueue_watcher -d "../config"
cd ..