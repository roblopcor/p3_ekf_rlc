# p3_ekf_rlc

A continuación se muestran los resultados de las distintas conficuraciones que pueden ser modificadas dentro de los scripts: "p3_ekf_adr/ekf_3d_state_estimation.py", "p3_ekf_adr/ekf_7d_state_estimation.py" y "p3_ekf_adr/ekf_8d_state_estimation.py".

En concreto se cambiará la variable  noise_case = "..." por las siguientes opciones:

- "base": caso de un ruido moderado tanto en la observación como en el del modelo
- "alta_observacion": ruido más alto en el las observaciones y un ruido moderado en el modelo
- "alto_modelo": ruido más alto en el modelo y moderado en las observaciones

## Modelo 3D (hex path)

| Caso base | Alta incertidumbre en la observación | Alta incertidumbre en el modelo de movimiento |
|:---------:|:---------------------------:|:------------------------------------:|
| <img src="imgs/hex_3d_base.png" width="250"> | <img src="imgs/alta_obs_3d.png" width="250"> | <img src="imgs/alto_mod_3d.png" width="250"> |
## Modelo 7D (circle path)

<div style="display: flex; justify-content: center; gap: 50px;">
  <img src="imgs/base_7d.png" alt="" width="260">
  <img src="imgs/alta_obs_7d.png" alt="" width="260">
  <img src="imgs/alto_modelo_7d.png" alt="" width="260">
</div> 

## Modelo 8d (default path)

<div style="display: flex; justify-content: center; gap: 50px;">
  <img src="imgs/base_8d.png" alt="" width="260">
  <img src="imgs/alta_obs_8d.png" alt="" width="260">
  <img src="imgs/alto_mod_8d.png" alt="" width="260">
</div> 
