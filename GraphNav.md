
<p align="center">
  <h1 align="center">Graph Nav</h1>
  <p align="center">

  </p>
</p>

# Überblick
GraphNav ist die Spot interne Methode zur Navigation in einer aufgezeichneten Umgebung. GraphNav ist ein Graphen basiertes Navigationsverfahren.
Ein Graph besteht dabei aus Wegpunkten und Kanten zwischen diesen Wegpunkten. Von Knoten zu Knoten ist dabei die Trasformation bekannt. Über die Kanten kann gespeichert werden, ob der nächste Wegpunkt über eine Treppe erreichbar ist.

# Kartenaufzeichnung 
Vorraussetzung für die Navigation über ein GraphNav Service ist die aufzeichnung einer Karte. Diese kann mithilfe der Spot-SDK erstelt werden. Für die Installation davon kann die [Anleitung](https://dev.bostondynamics.com/docs/python/quickstart.html).  durchgeführt werden.


Nach der erfolgreichen Installation kann mithilfe des **Command Line Record-Tools** eine Karte aufgezeichnet werden. Für die Benutzung steht die [Dokumentation](https://dev.bostondynamics.com/python/examples/graph_nav_command_line/readme)
 bereit.

# ROS Anbindung
Für die GraphNav-Navigation gibt es im Allgemeinen eine ROS-Anbindung durch den spot-ros-Treiber. DIeser Stellt die folgenden Services und Actions zu Verfügung.