# SGP41 ESPhome IDLE MODE

I've modified the existing source code to allow disabling the heater, which brings the idle power consumption down to under 50µA. For context, normal operation is around 4mA at startup and 3mA during measurement. There is also an option to disable the heater immediately upon device startup.

*YAML:
```
#-------------------------------------------
# EXTERNAL COMPONENT
#-------------------------------------------
external_components:
  - source: github://tiimsvk/sgp41-ESPhome-idle-mode@main
    components: [sgp41]
    
#-------------------------------------------
# SENSORS
#-------------------------------------------
sensor:
- platform: sgp4x
  voc:
    name: "VOC Index"
  nox:
    name: "NOx Index"
  compensation:
    humidity_source: dht1_hum
    temperature_source: dht1_temp
  heater_enable: True
  switch_power:
    name: "Power"
```

