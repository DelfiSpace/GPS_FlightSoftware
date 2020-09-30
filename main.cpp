#include <GPS.h>

// I2C bus
DWire I2Cinternal(0);
INA226 powerBus(I2Cinternal, 0x40);
TMP100 temp(I2Cinternal, 0x48);

// SPI bus
DSPI spi(3);
//MB85RS fram(spi, GPIO_PORT_P1, GPIO_PIN0, MB85RS::CY15B104QN50SXI);
MB85RS fram(spi, GPIO_PORT_P1, GPIO_PIN0, MB85RS::MB85RS1MT);


// HardwareMonitor
HWMonitor hwMonitor(&fram);

// Bootloader
Bootloader bootLoader = Bootloader(fram);

// CDHS bus handler
PQ9Bus pq9bus(3, GPIO_PORT_P9, GPIO_PIN0);

// SD CARD
DSPI_A SPISD;
SDCard sdcard(&SPISD, GPIO_PORT_P2, GPIO_PIN0);
LittleFS fs;

// services running in the system
FRAMService framServ(fram);
PingService ping;
ResetService reset( GPIO_PORT_P4, GPIO_PIN0, GPIO_PORT_P4, GPIO_PIN2 );

HousekeepingService<GPSTelemetryContainer> hk;
#ifndef SW_VERSION
SoftwareUpdateService SWupdate(fram);
#else
SoftwareUpdateService SWupdate(fram, (uint8_t*)xtr(SW_VERSION));
#endif

Service* services[] = { &ping, &reset, &hk, &SWupdate, &framServ };


// GPS board tasks
CommandHandler<PQ9Frame, PQ9Message> cmdHandler(pq9bus, services, 5);
PeriodicTask timerTask(1000, periodicTask);
PeriodicTask* periodicTasks[] = {&timerTask};
PeriodicTaskNotifier taskNotifier = PeriodicTaskNotifier(periodicTasks, 1);
Task* tasks[] = { &timerTask, &cmdHandler, &fs };

// system uptime
unsigned long uptime = 0;
FRAMBackedVar<unsigned long> totalUptime;

// TODO: remove when bug in CCS has been solved
void receivedCommand(DataFrame &newFrame)
{
    cmdHandler.received(newFrame);
}

void validCmd(void)
{
    reset.kickInternalWatchDog();
}

void periodicTask()
{
    // increase the timer, this happens every second
    uptime += 1;
    totalUptime += 1;

    // collect telemetry
    hk.acquireTelemetry(acquireTelemetry);

    // refresh the watch-dog configuration to make sure that, even in case of internal
    // registers corruption, the watch-dog is capable of recovering from an error
    reset.refreshConfiguration();

    // kick hardware watch-dog after every telemetry collection happens
    reset.kickExternalWatchDog();
    reset.kickInternalWatchDog();

    // pingFriends
//    pingModules();
//
//    retrieveCommCommandsReply();

}

void acquireTelemetry(GPSTelemetryContainer *tc)
{
    unsigned short v;
    signed short i, t;
    unsigned char uc;

    //HouseKeeping Header:
    tc->setStatus(Bootloader::getCurrentSlot());
    fram.read(FRAM_RESET_COUNTER + Bootloader::getCurrentSlot(), &uc, 1);
    tc->setBootCounter(uc);
    tc->setResetCause(hwMonitor.getResetStatus());
    tc->setUptime(uptime);
    tc->setTotalUptime((unsigned long) totalUptime);
    tc->setVersionNumber(2);
    tc->setMCUTemp(hwMonitor.getMCUTemp());

    // measure the power bus (INA226)
    tc->setINAStatus((!powerBus.getVoltage(v)) & (!powerBus.getCurrent(i)));
    tc->setVoltage(v);
    tc->setCurrent(i);

    // acquire board temperature (TMP100)
    tc->setTMPStatus(!temp.getTemperature(t));
    tc->setTemperature(t);
}

void printRX(){
    Console::log("RCF");
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
        {
            uint8_t RXData = MAP_UART_receiveData(EUSCI_A2_BASE);

            Console::log("%c", RXData);
        }
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);
}

/**
 * main.c
 */
void main(void)
{
    // initialize the MCU:
    // - clock source
    // - clock tree
    DelfiPQcore::initMCU();

    // initialize the ADC
    // - ADC14 and FPU Module
    // - MEM0 for internal temperature measurements
    ADCManager::initADC();

    // Initialize I2C master
    I2Cinternal.setFastMode();
    I2Cinternal.begin();

    // Initialize SPI master
    spi.initMaster(DSPI::MODE0, DSPI::MSBFirst, 1000000);

    //init FRAM and FRAM Variables
    fram.init();
    totalUptime.init(fram, FRAM_TOTAL_UPTIME, true, true);

    // initialize the shunt resistor
    powerBus.setShuntResistor(40);

    // initialize temperature sensor
    temp.init();

    // initialize the console
    Console::init( 115200 );     // baud rate: 9600 bps
    pq9bus.begin(115200, GPS_ADDRESS);     // baud rate: 115200 bps
                                 // address GPS (10)

    //InitBootLoader!
    bootLoader.JumpSlot();

    // initialize the reset handler:
    // - prepare the watch-dog
    // - initialize the pins for the hardware watch-dog
    // - prepare the pin for power cycling the system
    reset.init();

    // initialize Task Notifier
    taskNotifier.init();

    // initialize HWMonitor readings
    hwMonitor.readResetStatus();
    hwMonitor.readCSStatus();

    // link the command handler to the PQ9 bus:
    // every time a new command is received, it will be forwarded to the command handler
    // TODO: put back the lambda function after bug in CCS has been fixed
    //pq9bus.setReceiveHandler([](PQ9Frame &newFrame){ cmdHandler.received(newFrame); });
    pq9bus.setReceiveHandler(&receivedCommand);

    // every time a command is correctly processed, call the watch-dog
    // TODO: put back the lambda function after bug in CCS has been fixed
    //cmdHandler.onValidCommand([]{ reset.kickInternalWatchDog(); });
    //cmdHandler.onValidCommand(&validCmd);

    Console::log("GPS booting...SLOT: %d", (int) Bootloader::getCurrentSlot());

    if(HAS_SW_VERSION == 1){
        Console::log("SW_VERSION: %s", (const char*)xtr(SW_VERSION));
    }

    //sd detect
    MAP_GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN4);
    if(MAP_GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN4) == GPIO_INPUT_PIN_LOW){
        Console::log("SDCard Present");
        //Sd On
        Console::log("Configure SD-Card Pins");
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
        MAP_GPIO_setOutputHighOnPin( GPIO_PORT_P2, GPIO_PIN5);
        int err = sdcard.init();
        if(err){
        Console::log("SDCard Init: -%d",-err);
        }
        Console::log("Mounting SD....");

        fs.mount_async(&sdcard);
    }else{
        Console::log("SDCard NOT present");
    }

    eUSCI_UART_Config uartConfig;
    uartConfig.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    unsigned int n = MAP_CS_getSMCLK() / 115200;
    if (n > 16)
    {
        uartConfig.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION; // Over-sampling
        uartConfig.clockPrescalar = n >> 4;                                      // BRDIV = n / 16
        uartConfig.firstModReg = n - (uartConfig.clockPrescalar << 4);               // UCxBRF = int((n / 16) - int(n / 16)) * 16
    }
    else
    {
        uartConfig.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION; // Low-frequency mode
        uartConfig.clockPrescalar = n;                                            // BRDIV = n
        uartConfig.firstModReg = 0;                                               // UCxBRF not used
    }
    uartConfig.secondModReg = 32;    // UCxBRS = 32 http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html

    uartConfig.parity = EUSCI_A_UART_NO_PARITY;
    uartConfig.msborLsbFirst = EUSCI_A_UART_MSB_FIRST;
    uartConfig.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    uartConfig.uartMode = EUSCI_A_UART_MODE;

    Console::log("Set GPS UART Pins");
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                 GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    Console::log("Enable UART");
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);

    Console::log("Register Interrupt");
    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_registerInterrupt(EUSCI_A2_BASE, &printRX);

    Console::log("Enable GPS Module");
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);

    uint8_t enableNMEAdata[10] = {0xA0, 0xA1, 0x00 ,0x03 ,0x09, 0x01, 0x00, 0x08 ,0x0D, 0x0A};
    for(int i = 0; i < 10; i++){
        Console::log("sending: %x", enableNMEAdata[i]);
        MAP_UART_transmitData( EUSCI_A2_BASE, enableNMEAdata[i] );
    }


    TaskManager::start(tasks, 3);
}
