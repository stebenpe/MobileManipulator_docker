# Docker
Van het volledige project is ook een dockerfile gemaakt.
Deze dockerfile neemt de nieuwste github versie van het MoMa project op.
Dit is zowel de ros2 als de webserver kant.

# Docker container maken en runnen
De docker file kan worden gecreëerd door de docker_build.sh te runnen. Dit moet wel in dezelfde map als waar de dockerfile zit.
De docker_build.sh kan op de volgende manier worden uitgevoerd:
``` 
./docker_build.sh
```
vervolgens kan de docker container worden gestart door het volgende commando uit te voeren:
```
./docker_run.sh
```
Dit start de container met alle juiste instellingen.

# Dockerfile aanpassen
De dockerfile kan naar behoefte worden aangepast. Zo kan de hoofd image worden veranderd naar een amd in plaats van arm architecture. Ook kan de entrypoint worden veranderen naar een headless versie. Deze versie start bij het runnen van de container meteen de ros2 drivers en webserver op de achtergrond op.