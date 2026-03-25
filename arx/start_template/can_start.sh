#!/bin/bash
# gnome-terminal -t "can" -- bash -c "cd ~/ARX_X5/ARX_CAN && ./can.sh;exec bash;"

gnome-terminal --window -e 'bash -c "cd /home/kai/ARX_X5/ARX_CAN/arx_can; echo kai0717 | sudo -S ./arx_can0.sh; exec bash;"' \
--tab -e 'bash -c "sleep 1; cd /home/kai/ARX_X5/ARX_CAN/arx_can; echo kai0717 | sudo -S ./arx_can1.sh; exec bash;"' \
--tab -e 'bash -c "sleep 2; cd /home/kai/ARX_X5/ARX_CAN/arx_can; echo kai0717 | sudo -S ./arx_can2.sh; exec bash;"' \
--tab -e 'bash -c "sleep 3; cd /home/kai/ARX_X5/ARX_CAN/arx_can; echo kai0717 | sudo -S ./arx_can3.sh; exec bash;"'


