cd ~/ut_automata
pkill -P $$
roscore &
./bin/simulator --localize &
./bin/websocket & 
