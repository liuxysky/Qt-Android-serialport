/****************************************************************************
**
** Copyright (C) 2014 jpgk.Co.Ltd. All rights reserved.
** author   : Liu Xiangyu liuxysky@126.com
** version  : 1.1
** file     : androidserialport.h
** explain  : Use the serialport in C++
** 说明     : 在C++中使用串口，原理是调用linux函数来操作串口，使用时不要连qt的串口库
**            用法与使用qt串口类相同，但是参数必须用本程序提供的枚举类型
** 版本说明  ： 1.1版修改错误信息打印，可以将系统错误信息通过qdebug打印出来
**
**
****************************************************************************/

#ifndef ANDROIDSERIALPORT_H
#define ANDROIDSERIALPORT_H

#include <QObject>
#ifdef Q_OS_LINUX
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#endif

#include <QtCore/qsocketnotifier.h>
#include <QTimer>
#include <QThread>


class ReadNotifier;
class AndroidSerialPort;



class AndroidSerialPort : public QObject
{
    Q_OBJECT
public:

    enum OpenMode {
        NotOpen = 0x0000,
        ReadOnly = 0x0001,
        WriteOnly = 0x0002,
        ReadWrite = ReadOnly | WriteOnly,
        Append = 0x0004,
        Truncate = 0x0008,
        Text = 0x0010,
        Unbuffered = 0x0020
    };
    enum Directions  {
        Input = 1,
        Output = 2,
        AllDirections = Input | Output
    };


    enum BaudRate {
        Baud1200 = B1200,
        Baud2400 = B2400,
        Baud4800 = B4800,
        Baud9600 = B9600,
        Baud19200 = B19200,
        Baud38400 = B38400,
        Baud57600 = B57600,
        Baud115200 = B115200,
        UnknownBaud = -1
    };

    enum DataBits {
        Data5 = 5,
        Data6 = 6,
        Data7 = 7,
        Data8 = 8,
        UnknownDataBits = -1
    };

    enum Parity {
        NoParity = 0,
        EvenParity = 2,
        OddParity = 3,
        SpaceParity = 4,
        MarkParity = 5,
        UnknownParity = -1
    };

    enum StopBits {
        OneStop = 1,
        OneAndHalfStop = 3,
        TwoStop = 2,
        UnknownStopBits = -1
    };

    enum FlowControl {
        NoFlowControl,
        HardwareControl,
        SoftwareControl,
        UnknownFlowControl = -1
    };

    enum PinoutSignal {
        NoSignal = 0x00,
        TransmittedDataSignal = 0x01,
        ReceivedDataSignal = 0x02,
        DataTerminalReadySignal = 0x04,
        DataCarrierDetectSignal = 0x08,
        DataSetReadySignal = 0x10,
        RingIndicatorSignal = 0x20,
        RequestToSendSignal = 0x40,
        ClearToSendSignal = 0x80,
        SecondaryTransmittedDataSignal = 0x100,
        SecondaryReceivedDataSignal = 0x200
    };
    Q_DECLARE_FLAGS(PinoutSignals, PinoutSignal)

    enum DataErrorPolicy {
        SkipPolicy,
        PassZeroPolicy,
        IgnorePolicy,
        StopReceivingPolicy,
        UnknownPolicy = -1
    };

    enum SerialPortError {
        NoError,
        DeviceNotFoundError,
        PermissionError,
        OpenError,
        ParityError,
        FramingError,
        BreakConditionError,
        WriteError,
        ReadError,
        ResourceError,
        UnsupportedOperationError,
        UnknownError
    };

    explicit AndroidSerialPort(QObject *parent = 0);
    ~AndroidSerialPort();
    int fileFd;
    bool readNotification();
signals:
    void readyRead();

public slots:
    void setPortName(const QString &name);
    inline QString portName() const
    {return thePortName;}

    bool open(AndroidSerialPort::OpenMode mode) ;
    void close() ;

    bool setBaudRate(BaudRate baudRate, Directions dir = AllDirections);
    inline BaudRate baudRate(Directions dir = AllDirections) const
    {if(dir == AllDirections)return theBaudRate;return theBaudRate;}

    bool setDataBits(DataBits dataBits);
    inline DataBits dataBits() const
    {return theDataBits;}

    bool setParity(Parity parity);
    inline Parity parity() const
    {return theParity;}

    bool setStopBits(StopBits stopBits);
    inline StopBits stopBits() const
    {return theStopBits;}

    bool setFlowControl(FlowControl flow);
    inline FlowControl flowControl() const
    {return theFlowControl;}

    bool clear(Directions dir = AllDirections);

    bool waitForReadyRead(int msecs) ;
    bool waitForBytesWritten(int msecs) ;

    inline bool isOpen() const
    {return openStat;}

//    void deleteLater();

    qint64 read(char *data, qint64 maxlen);
    QByteArray read(qint64 maxlen);
    QByteArray readAll();

    qint64 write(const char *data, qint64 len);
    qint64 write(const char *data);
    qint64 write(const QByteArray &data);

    inline qint64 readBufferSize() const
    {return readBufferMaxSize;}

    void emitReadyRead();

private:
    bool updateTermios();



    ssize_t thewrite();
    ssize_t theRead();

    QByteArray readBuffer;
    QByteArray writeBuffer;

    QString thePortName;
    bool openStat;

    BaudRate theBaudRate;
    DataBits theDataBits;
    Parity theParity;
    StopBits theStopBits;
    FlowControl theFlowControl;

    qint64 readBufferMaxSize;

    struct termios currentTermios;
    struct termios restoredTermios;

    bool emittedReadyRead;

    QSocketNotifier *readNotifier;

    QTimer *timer;
};

#endif // ANDROIDSERIALPORT_H
