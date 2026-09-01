# CG4002 FireBeetle Code

## Workflow

### Data preprocessing

1. (Read sensor once) 12 raw bytes received. 

2. Parse into 6 16-bit int raw values by concatenating every 2 bytes, which are the raw numbers from sensor. 

3. Read sensor for 3 seconds and get 300 (tbc) readings by repeating steps 1 & 2, each reading is an array of 6 int16_t. This huge array will have 300 rows and 6 cols. 

4. Start preprocessing (for ML input). First introduce calibrations. Second do median spike removal. 

5. Send out final data (300x6 int16_t). Reset all buffers.

## Codebases

1. Dummy code

Generate random fake IMU data at high frequency. Simulate data collection and storage. 

2. Data collection code

Use I2C to read real data from an IMU to replace dummy data. 

3. Real data code

To be updated. 

## Helpful info

1. IMU registers

0x22  Gyro X L
0x23  Gyro X H
0x24  Gyro Y L
0x25  Gyro Y H
0x26  Gyro Z L
0x27  Gyro Z H
0x28  Accel X L
0x29  Accel X H
0x2A  Accel Y L
0x2B  Accel Y H
0x2C  Accel Z L
0x2D  Accel Z H

