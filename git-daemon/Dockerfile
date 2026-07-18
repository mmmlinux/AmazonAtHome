FROM alpine:3.19

RUN apk add --no-cache git

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 9418

ENTRYPOINT ["/entrypoint.sh"]
