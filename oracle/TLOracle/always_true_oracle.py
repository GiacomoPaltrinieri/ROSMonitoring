#!/usr/bin/env python3

import argparse
import json

from websocket_server import WebsocketServer


def new_client(client, server): #chiamata quando mon collegato
    print("New ROS monitor connected and was given id %d" % client['id'])


def client_left(client, server):#chiamata quando mon si scollega
    print("ROS monitor (%d) disconnected" % client['id'])


def message_received(client, server, message):
    try:
        message_dict = json.loads(message) #convesione a diz
    except json.JSONDecodeError:
        message_dict = {}
    message_dict['verdict'] = 'currently_true' #non controlla ma aggiunge in automatico il verdetto a true
    server.send_message(client, json.dumps(message_dict)) #invia messaggio


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=8080, type=int) 
    args = parser.parse_args()

    server = WebsocketServer(args.port)
    server.set_fn_new_client(new_client)
    server.set_fn_client_left(client_left)
    server.set_fn_message_received(message_received)
    server.run_forever()


if __name__ == '__main__':
    main()
