# Código do MOOS-IvP para pesquisa de mestrado

## Módulos

### `pContactSpawn`  
Gera contatos que virão em direção do navio de maneira simulada quando determinadas variáveis forem escolhidas.  

- Os contatos são gerados de forma a **simular as regras do RIPEAM** que queremos testar.

---

### `pAstarColAvd`  
Responsável por realizar o **collision avoidance** desse contato utilizando o algoritmo **A\*** e também respeitando as regras do RIPEAM.  

---

## Comando para gerar um contato

```bash
uPokeDB SPAWN_CONTACT="heading=270,relative_bearing=0,distance=500,speed=5"
```

## Rodando as missões

Basicamente temos 3 missões, cada uma para um tipo de algoritmo:

col_avd_moos -> Algoritmo padrão do MOOS-IvP
astar -> Algoritmo A* desenvolvido
velocity_obstacle -> Algoritmo de VO 
