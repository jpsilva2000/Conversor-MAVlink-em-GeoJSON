# Conversor-MAVlink-em-GeoJSON

O objetivo dos seguintes scripts é criar um ficheiro ".json" através da leitura de mensagens GLOBAL_POSITION_INT, estes foram desenvolvidos com recurso ao pymvalink. 

-> O script "heatbeats_plus_position.py" envia Heartbeats e uma Posição fixa, simbolizando o UAV/UGV. O envio das mensagens GLOBAL_POSITION_INT será utilizado para testar a funcionalidade do script conversor. 

-> No lado da estação de controlo, com recurso ao pymavproxy, é usado o seguinte comando: mavproxy.py --master=udpout:"IP_UAV/UGV":14550 --out=udp:127.0.0.1:14551

Com este comando, a ligação está estabelecida e a estação está a fazer forwarding das mensagens para o porto 14551, no localhost. 


-> Correndo o script "conversor_bms_json.py" este recebe as mensagens e efetua a "tradução" para JSON, depois disto é guardado o output num ficheiro JSON e enviado por MQTT para um servidor. Além disso também está à escuta de mensagens GeoJSON no servidor, traduz para MAVlink, e faz o envio das mesmas como mensagens ADSB.
