# MazeFinder

Project for [Introduction to Artificial Intelligence](https://kam.fit.cvut.cz/bi-zum/) where group of robots are locating 
mazes inside labyrinth and then they are transported to the depo. Robot can carry only one maze at a time and collisions
are not taken into account for simplicity. Exploration of labyrinth is done using BFS, but Robot always prioritize move
which will contribute most to the exploration. Transportation of mazes is done via AStar algorithm. PyGame is utilised
for visualization.

![example](example.gif)