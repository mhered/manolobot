# HRB LiPo Battery 

## Battery specs

Capacity: 5000mAh 

Voltage: 11.1V 3S 

Discharge rate: 50C 

XT60 Connector with 4 adapter parts. 

Balance connector: JST-XH

Carbon fibre rigid case: 155×44×24mm (0 - 3mm Unterschied) 

Weight 376 g

Safe charging rate: 0.5-1C (i.e. in my case 5A)

## Rationale for the choice of battery

cfr: https://articulatedrobotics.xyz/mobile-robot-5-power-theory/

### Requirements

#### Consumers @5V:

* RPi 1.5A
* Arduino 0.1A
* Lidar 1A
* PiCamera 0.1A
* Motor driver  0.1A
* LCD 1A

Total: 3.8A @5V --> 1.6A@12V --> 2A 

#### Consumers @12V

2 motors x1.5A = 3A

#### Total requirement

5A @12V during 40'

### Explanation of the specs

#### S-number

e.g. 2S 3S 4S is the number of 3,7V cells in series

3.7 is the storage level, fully charged each cell is 4.2V

e.g. in our case:**NEED 3 (11.1V avg)** 

#### Capacity in mAh

The maximum continuous current in mA that fully discharges the battery over 1h. 

Measures the run time from a single charge, but in practice one should use maximum 2/3 of the charge.

Therefore minutes to safe discharge @5A = CAPACITY mAh/ (5A * 1000mA/A) * 60min(h) *2/3 = 0.008 * CAPACITY

i.e. in our case to achieve approx 40' usable time @5A: **NEED 5000mAh** 

#### C-number

The discharge rate is how fast you can discharge the battery without harming it. 

C-number is the maximum current expressed as a multiplier of the capacity. Typically two values: continuous and burst.

e.g. in our case **50C** * 5000mAH *1A/1000mA= 250A **(more than enough!)** 

## Battery basics

cfr : [Battery basics](https://www.youtube.com/watch?v=4bzyvmiFTZo), [LiPo guide for beginners](https://www.youtube.com/watch?v=Lk7wzVYmXSA)

### Pros/Cons LiPo vs NiCad, NiMH etc

Pros: Lighter, higher capacity, high discharge, various sizes

Cons: Shorter life (100-200 cycles), risk of fire

### Safety

* Never use a puffed or damaged battery
* Never leave a charging battery unattended. Never leave a battery plugged in (either in the robot or in the charger)

* Never charge any cell above 4.2V or below 3.0V

* Never leave a Lipo batttery fully charged or fully dischared more than 3 days: charge them to storage level (3.7V per cell)
* Never immediately use or charge a warm LiPo battery
* Never store batteries in a hot environment. Heat kills batteries.

### Charging

See [How to charge lithium batteries](https://www.makeuseof.com/how-to-charge-a-lipo-battery/ ) for the charger I bought.

Recommended to charge slowly (at 1C)

4 settings: Charge, Balance, Storage, Discharge

* **Charge** may lead one individual cell to overcharge
* Use **Balance** to increase the life, uses **balance connector** to ensure all cells are charged evenly (). Different connectors for 2S, 3S 4S cells. 

* Use **Storage** to charge to 3.7V per cell if you wont use the batteries for months

### Storage

Store in a tin box or lipo pouches

### Connectors

XT60 connector is better than older T or Dean's style connector

Balance connector: to ensure all cells are charged equally. Different connectors for 2S, 3S 4S cells