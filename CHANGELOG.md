# Changelog for sdk

## 1.1 (2023-4-25)
------------------
- add function of Power rail,fan speed,charge config,firmware version query without firmware version configuration and AMR Node config
- Contributors: Junyu Luo

## 1.1 (2023-4-24)
------------------
- test all feedbacks of 0x2 and 0x3
- add factory setting function,clear error state function,acc and dec config
- Contributors: Junyu Luo

## 1.1 (2023-4-20)
------------------
- test light rgb command message using virtual can network.
- test other 0x1 series command
- Contributors: Junyu Luo

## 1.1 (2023-4-18)
------------------
- test motion command and light mode control command using virtual can network,this two functionality is ok
- remove other agilex robot cpps and hpps
- add reserve struct into motion command message because if not the byte[4]-byte[7] will be random number
- Contributors: Junyu Luo

## 1.0 (2023-04-8)
-------------------
* Add percy protocol messages and ids into project without tests
* Contributors: Junyu Luo