#!/bin/bash

if [ $(id -u) -ne 0 ]; then
  echo "This script must be run as root";
  exit 1;
fi

echo 'Disable proxy'

# gsettings set org.gnome.system.proxy mode 'none' ;

grep PATH /etc/environment > lol.t;
cat lol.t > /etc/environment;

printf "" > /etc/apt/apt.conf.d/95proxies;

git config --global http.proxy ""

rm -rf lol.t;
