#!/bin/bash

tar --exclude='.git' -zcvf submit/aichallenge_submit.tar.gz -C ./aichallenge/workspace/src aichallenge_submit
