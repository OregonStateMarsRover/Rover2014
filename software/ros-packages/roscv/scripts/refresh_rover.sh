#/bin/bash
git pull
echo "Type rover password"
read -s PASSWORD
git push rover master
