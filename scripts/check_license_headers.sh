#!/bin/bash

# Ignore patterns not matching anything
shopt -s nullglob 

src_dirs=( "src" "test" )
endings=( "cpp" "hpp" )
error=false

for dir in ${src_dirs[@]}
do
    for ending in ${endings[@]}
    do
	for file in "$dir"/*."$ending" "$dir"/**/*."$ending"
	do
	    grep Copyright "$file" > /dev/null 
	    if [[ $? -gt 0 ]]; then
	       echo "file ${file} does not have a Copyright notice"
	       error=true
	    fi
	done
    done
done

if [[ $error = true ]]; then
   exit 1
fi
