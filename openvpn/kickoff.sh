#!/bin/sh

echo "Kicking OpenVPN client extension"
openvpn /persist/openvpn/test.ovpn
echo "We've crashed or could not locate the .ovpn file under /persist/openvpn/"
echo "Please scp the .ovpn file to /persist/openvpn/*.ovpn and try again"