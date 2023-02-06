#! /usr/bin/env bash

cat cancoder-potentiometer-data-raw.tsv | jq -R \
	'[ inputs | split("\t") | { encoderAngle: .[0], rotations: .[1], potentiometerVoltage: .[2], idealVoltage: .[3] } ]' >	cancoder-potentiometer-data-raw.json
