#!/bin/bash

if [ $(id -u) -ne 0 ]; then
  echo "This script must be run as root";
  exit 1;
fi

$(dirname $0)/proxy_off.sh

phost="10.128.0.100";
pport=8080;

echo "Setup: http://$phost:$pport/"

# gsettings set org.gnome.system.proxy mode 'manual' ;
# gsettings set org.gnome.system.proxy.http host $phost;
# gsettings set org.gnome.system.proxy.http port $pport;

grep PATH /etc/environment > lol.t;
printf \
"http_proxy=http://$phost:$pport/\n\
https_proxy=http://$phost:$pport/\n\
ftp_proxy=http://$phost:$pport/\n\
no_proxy=\"localhost,127.0.0.1,localaddress,.localdomain.com\"\n\
HTTP_PROXY=http://$phost:$pport/\n\
HTTPS_PROXY=http://$phost:$pport/\n\
FTP_PROXY=http://$phost:$pport/\n\
NO_PROXY=\"localhost,127.0.0.1,localaddress,.localdomain.com\"\n" >> lol.t;

cat lol.t > /etc/environment;

printf \
"Acquire::http::proxy \"http://$phost:$pport/\";\n\
Acquire::ftp::proxy \"ftp://$phost:$pport/\";\n\
Acquire::https::proxy \"https://$phost:$pport/\";\n" > /etc/apt/apt.conf.d/95proxies;

git config --global http.proxy "10.128.0.100:8080"

rm -rf lol.t;

