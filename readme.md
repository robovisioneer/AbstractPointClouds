<h1 align="center">Hy 👋, ich bin Christian Wiegel</h1>
<p align="left">
Fuer meine Masterarbeit habe ich Punktwolken von Koerpern mit dem SHOT-Decriptor der Point Cloud Library (PCL) analysiert. Vergleicht man die SHOT-Descriptoren zweier Punktwolken, kann man die Aehnlichkeit der beiden Koerper berechnen. Das Programm gibt den Unterschied / die Differenz aus. Diese Groesse ist gegenläufig zur Ähnlichkeit. Zwei Punktwolken vom selben Koerper haben einen Unterschied / eine Differenz von 0.
</p>
<p align="left">
Ich habe mich immer gefragt, was dabei herauskommt, wenn man mit abstrakten geometrischen Koerpern arbeitet. Ich habe deswegen Punktewolken von abstrakten geometrischen Körpern erstellt:
</p>

<p align="left">
Eine Kugel:
</p>

<p align="left">
<img src="https://github.com/robovisioneer/AbstractPointClouds/blob/main/PCL_Kugel.png" alt="cplusplus" width="100" height="100"/>
</p>

<p align="left">
Ein Kegel:
</p>

<p align="left">
<img src="https://github.com/robovisioneer/AbstractPointClouds/blob/main/PCL_Kegel.png" alt="cplusplus" width="100" height="100"/>
</p>

<p align="left">
Ein Zylinder:
</p>

<p align="left">
<img src="https://github.com/robovisioneer/AbstractPointClouds/blob/main/PCL_Zylinder.png" alt="cplusplus" width="100" height="100"/>
</p>

<p align="left">
Die Unterschiede ergeben sich wie folgt:
</p>

<p align="left">
<table border="1">
    <thead>
        <th></th>
        <th>Kegel</th>
        <th>Zylinder</th>
    </tr>
    </thead>
    <tbody>
        <th>Kugel</th>
        <td>621,34</td>
        <td>773,524</td>
    </tr>
    <tr>
        <th>Kegel</th>
        <td>0</td>
        <td>406,936</td>
    </tr>
  </tbody>
</table>
</p>

<p align="left">
Dass der Unterschied zwischen Kugel und Zylinder am groessten und der Unterschied zwischen Kegel und Zylinder am geringsten ausfaellt, macht für mich sehr viel Sinn.
</p>

<p align="left">
Der SHOT-Descriptor ist auf gekruemmte Flaechen angewiesen; von einem idealen Wuerfel, laesst er sich nicht berechnen. Pyramiden und Tetraeder habe ich daher gar nicht erst probiert.
</p>

<h3 align="left">Languages and Tools:</h3>
<p align="left"> <a href="https://www.w3schools.com/cpp/" target="_blank" rel="noreferrer"> <img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/cplusplus/cplusplus-original.svg" alt="cplusplus" width="40" height="40"/> </a> <a href="https://opencv.org/" target="_blank" rel="noreferrer"> <img src="https://www.vectorlogo.zone/logos/opencv/opencv-icon.svg" alt="opencv" width="40" height="40"/> </a> <a href="https://pointclouds.org/" target="_blank" rel="noreferrer"> <img src="https://github.com/robovisioneer/AbstractPointClouds/blob/main/logo.png" alt="pcl" width="126" height="40"/> </a> </p>