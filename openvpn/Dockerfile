FROM arm64v8/alpine

RUN apk update
RUN apk add openvpn
RUN mkdir -p /dev/net; mknod /dev/net/tun c 10 200; chmod 600 /dev/net/tun

COPY kickoff.sh /app/
WORKDIR /app
RUN chmod +x /app/kickoff.sh
ENTRYPOINT ["/app/kickoff.sh"]