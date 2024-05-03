#!/bin/bash
# ghcrにdocker loginするためのスクリプト
# ~/.netrcにgithub向けのPersonal Access Tokenを置いたら、そこから情報を取ってくる。

FILE_NAME=~/.netrc
if [ ! -e $FILE_NAME ];then
    echo $FILE_NAME がありません。
    echo GitHubでPersonal Access Tokenを取得し、 $FILE_NAME に設定してください。
    exit 1
fi

while read line
do
    data=($line)
    if [ ${data[0]:-default} == "login" ];then
        GITHUB_USER_NAME=${data[1]}
    elif [ ${data[0]:-default} == "password" ];then
        GITHUB_PAT=${data[1]}
    fi
done < $FILE_NAME

echo $FILE_NAME から取得した、以下の情報をghcr.ioへのログインに使用します
echo username: $GITHUB_USER_NAME
echo personal access token: ${GITHUB_PAT:0:5}**********

echo $GITHUB_PAT | docker login ghcr.io -u $GITHUB_USER_NAME --password-stdin
