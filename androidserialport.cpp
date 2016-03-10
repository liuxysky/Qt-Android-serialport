/****************************************************************************
**
** Copyright (C) 2014 jpgk.Co.Ltd. All rights reserved.
** author : Liu Xiangyu liuxysky@126.com
** version: 1.1
** file   : androidserialport.cpp
**
****************************************************************************/

#include "androidserialport.h"

class ReadNotifier : public QSocketNotifier
{
public:
    ReadNotifier(AndroidSerialPort *d, QObject *parent = 0)
        : QSocketNotifier(d->fileFd, QSocketNotifier::Read, parent)
        , dptr(d)
    {}

protected:
    bool event(QEvent *e) Q_DECL_OVERRIDE {
        bool ret = QSocketNotifier::event(e);
        if (ret)
            dptr->readNotification();
        return ret;
    }

private:
    AndroidSerialPort *dptr;
};


AndroidSerialPort::AndroidSerialPort(QObject *parent) :
    QObject(parent)
{
    openStat = false;
    fileFd = -1;
    readBufferMaxSize = 10240;
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(emitReadyRead()));


}

AndroidSerialPort::~AndroidSerialPort()
{
    ::close(fileFd);
}

void AndroidSerialPort::setPortName(const QString &name)
{
    thePortName = name;
}

bool AndroidSerialPort::open(AndroidSerialPort::OpenMode mode)
{
    if(thePortName.size() == 0){
        qDebug("LiuXY: need set port name first!");
        return false;
    }
    QByteArray portName = thePortName.toLocal8Bit();
    const char *ptr = portName.constData();

    int flags = O_NOCTTY | O_NONBLOCK;

    switch (mode & ReadWrite) {
    case WriteOnly:
        flags |= O_WRONLY;
        break;
    case ReadWrite:
        flags |= O_RDWR;
        break;
    default:
        flags |= O_RDONLY;
        break;
    }

    fileFd = ::open(ptr, flags);

    if (fileFd == -1) {
        qDebug("LiuXY: can't open this sairal port %s", ptr);
        qDebug("LiuXY: open: %s", strerror(errno));/*perror("open");*/
        return false;
    }

    ::fcntl(fileFd, F_SETFL, O_NDELAY);

#ifdef TIOCEXCL
    ::ioctl(fileFd, TIOCEXCL);
#endif

    if (::tcgetattr(fileFd, &restoredTermios) == -1) {
        qDebug("LiuXY: can't get this termios stat");
        close();
        return false;
    }

    currentTermios = restoredTermios;
    ::cfmakeraw(&currentTermios);
    currentTermios.c_cflag |= CLOCAL;
    currentTermios.c_cc[VTIME] = 0;
    currentTermios.c_cc[VMIN] = 0;

    if (mode & ReadOnly)
        currentTermios.c_cflag |= CREAD;

    if (!updateTermios()){
        close();
        return false;
    }

    openStat = true;

    //TODO add the probe read stat;
    readNotifier = new ReadNotifier(this);
    readNotifier->setEnabled(true);
    return true;
}

void AndroidSerialPort::close()
{
    ::close(fileFd);
    fileFd = -1;
    openStat = false;
}

bool AndroidSerialPort::setBaudRate(AndroidSerialPort::BaudRate baudRate, AndroidSerialPort::Directions dir)
{
    bool ret = baudRate > 0;

    // prepare section

    if (ret) {
        theBaudRate = baudRate;
        const qint32 unixBaudRate = baudRate;
        if (unixBaudRate > 0) {
            // prepare to set standard baud rate
            ret = !(((dir & Input) && ::cfsetispeed(&currentTermios, unixBaudRate) < 0)
                    || ((dir & Output) && ::cfsetospeed(&currentTermios, unixBaudRate) < 0));
        }
    }

    // finally section
    if (ret) // finally, set baud rate
        ret = updateTermios();
    else
        qDebug("LiuXY: can't set baud rate\n");
    return ret;
}

bool AndroidSerialPort::setDataBits(AndroidSerialPort::DataBits dataBits)
{
    theDataBits = dataBits;
    currentTermios.c_cflag &= ~CSIZE;
    switch (dataBits) {
    case Data5:
        currentTermios.c_cflag |= CS5;
        break;
    case Data6:
        currentTermios.c_cflag |= CS6;
        break;
    case Data7:
        currentTermios.c_cflag |= CS7;
        break;
    case Data8:
        currentTermios.c_cflag |= CS8;
        break;
    default:
        currentTermios.c_cflag |= CS8;
        break;
    }
    return updateTermios();
}

bool AndroidSerialPort::setParity(AndroidSerialPort::Parity parity)
{
    theParity = parity;
    currentTermios.c_iflag &= ~(PARMRK | INPCK);
    currentTermios.c_iflag |= IGNPAR;

    switch (parity) {

#ifdef CMSPAR
    // Here Installation parity only for GNU/Linux where the macro CMSPAR.
    case SpaceParity:
        currentTermios.c_cflag &= ~PARODD;
        currentTermios.c_cflag |= PARENB | CMSPAR;
        break;
    case MarkParity:
        currentTermios.c_cflag |= PARENB | CMSPAR | PARODD;
        break;
#endif
    case NoParity:
        currentTermios.c_cflag &= ~PARENB;
        break;
    case EvenParity:
        currentTermios.c_cflag &= ~PARODD;
        currentTermios.c_cflag |= PARENB;
        break;
    case OddParity:
        currentTermios.c_cflag |= PARENB | PARODD;
        break;
    default:
        currentTermios.c_cflag |= PARENB;
        currentTermios.c_iflag |= PARMRK | INPCK;
        currentTermios.c_iflag &= ~IGNPAR;
        break;
    }

    return updateTermios();
}

bool AndroidSerialPort::setStopBits(AndroidSerialPort::StopBits stopBits)
{
    theStopBits = stopBits;
    switch (stopBits) {
    case OneStop:
        currentTermios.c_cflag &= ~CSTOPB;
        break;
    case TwoStop:
        currentTermios.c_cflag |= CSTOPB;
        break;
    default:
        currentTermios.c_cflag &= ~CSTOPB;
        break;
    }
    return updateTermios();
}

bool AndroidSerialPort::setFlowControl(AndroidSerialPort::FlowControl flow)
{
    theFlowControl = flow;
    switch (flow) {
    case NoFlowControl:
        currentTermios.c_cflag &= ~CRTSCTS;
        currentTermios.c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
    case HardwareControl:
        currentTermios.c_cflag |= CRTSCTS;
        currentTermios.c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
    case SoftwareControl:
        currentTermios.c_cflag &= ~CRTSCTS;
        currentTermios.c_iflag |= IXON | IXOFF | IXANY;
        break;
    default:
        currentTermios.c_cflag &= ~CRTSCTS;
        currentTermios.c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
    }
    return updateTermios();
}

bool AndroidSerialPort::clear(AndroidSerialPort::Directions dir)
{
    if(dir == AllDirections){
        readBuffer.clear();
        writeBuffer.clear();
    }
    if(dir == Input){
        readBuffer.clear();
    }
    if(dir == Output){
        writeBuffer.clear();
    }
    return true;
}

bool AndroidSerialPort::waitForReadyRead(int msecs)
{
//    qDebug("file fd %d", fileFd);
    struct timeval tv;

    tv.tv_sec = msecs / 1000;
    tv.tv_usec = (msecs % 1000) * 1000;

    fd_set rfds;

    int retval;

    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(fileFd, &rfds);

    //FD_ISSET( fd, &rfds );
    retval = select(fileFd + 1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */

    if (retval == -1){
        qDebug("LiuXY: select :%s", strerror (errno));
//        perror("select()");
        return false;
    }
    else if (retval){
//        qDebug("Data is available now. %d", retval);
//        readNotification();
        while(theRead()){
            QThread::msleep(10);
        }
        return true;
    }
    /* FD_ISSET(0, &rfds) will be true. */
    else
        printf("No data within five seconds.\n");
    return false;

}

bool AndroidSerialPort::waitForBytesWritten(int msecs)
{
    if(msecs<0)
        return false;
    if(writeBuffer.size() == 0){
        return true;
    }
    while(writeBuffer.size() > 0){
        int len = thewrite();
        writeBuffer.remove(0, len);
    }
    return true;
}

qint64 AndroidSerialPort::read(char *data, qint64 maxlen)
{
//    theRead();
    if(data == NULL || maxlen < 0){
        return -1;
    }
    if(maxlen == 0)
        return 0;
    qint64 returnlen = 0;
    if(maxlen < readBuffer.size()) {
        QByteArray temp = readBuffer.left(maxlen);
        readBuffer.remove(0, maxlen);
        memcpy(data, temp.data(), temp.size ());
        returnlen = temp.size();
    }
    else {
        memcpy(data, readBuffer.data(), readBuffer.size ());
        returnlen = readBuffer.size();
        readBuffer.clear();
    }
    return returnlen;
}

QByteArray AndroidSerialPort::read(qint64 maxlen)
{
    QByteArray returndata;
//    theRead();

    if(maxlen < readBuffer.size()) {
        returndata = readBuffer.left(maxlen);
        readBuffer.remove(0, maxlen);
    }
    else {
        returndata.append(readBuffer.data(), readBuffer.size()); //TODO
        readBuffer.clear();
    }
    return returndata;
}

QByteArray AndroidSerialPort::readAll()
{
    QByteArray returndata;
//    theRead();
    returndata.append(readBuffer.data(), readBuffer.size()); //TODO
    readBuffer.clear();
    return returndata;
}

qint64 AndroidSerialPort::write(const char *data, qint64 len)
{
    qint64 returnlen;
    writeBuffer.append(data, len);
    if(writeBuffer.size() != len)
        return -1;
    returnlen = thewrite();
    writeBuffer.clear();
    return returnlen;
}

qint64 AndroidSerialPort::write(const char *data)
{
    qint64 returnlen;
    int len = strlen(data);
    writeBuffer.append(data, len);
    if(writeBuffer.size() != len)
        return -1;
    returnlen = thewrite();
    writeBuffer.clear();
    return returnlen;
}

qint64 AndroidSerialPort::write(const QByteArray &data)
{
    qint64 returnlen;
    writeBuffer.append(data.data(), data.size()); //TODO
    returnlen = thewrite();
    writeBuffer.clear();
    return returnlen;
}

void AndroidSerialPort::emitReadyRead()
{
    if(!emittedReadyRead){
        emittedReadyRead = true;
        emit readyRead();
        timer->stop();
        emittedReadyRead = false;
    }
}

bool AndroidSerialPort::updateTermios()
{
    if (::tcsetattr(fileFd, TCSANOW, &currentTermios) == -1) {
        qDebug("LiuXY: can't set termios");
        return false;
    }
    return true;
}

bool AndroidSerialPort::readNotification()
{
//    qDebug("read function");
    qint64 newBytes = readBuffer.size();
    qint64 bytesToRead = 1;

    if (readBufferMaxSize && bytesToRead > (readBufferMaxSize - readBuffer.size())) {
        bytesToRead = readBufferMaxSize - readBuffer.size();
        if (bytesToRead == 0) {
            // Buffer is full. User must read data from the buffer
            // before we can read more from the port.
            return false;
        }
    }

    const qint64 readBytes = theRead();

    if (readBytes <= 0) {

        return false;
    }
    else if(readBytes > 0){
        while(theRead());
    }

    newBytes = readBuffer.size() - newBytes;

    // If read buffer is full, disable the read port notifier.
    if (readBufferMaxSize && readBuffer.size() == readBufferMaxSize)
        readNotifier->setEnabled(false);

    // only emit readyRead() when not recursing, and only if there is data available
    const bool hasData = newBytes > 0;

    if (hasData) {
        timer->start(10);
    }

    if (!hasData){
        readNotifier->setEnabled(true);
    }



    return true;
}

ssize_t AndroidSerialPort::thewrite()
{
    return ::write(fileFd, writeBuffer.data(), writeBuffer.size());
}

ssize_t AndroidSerialPort::theRead()
{
    int len = 0;
    char readData[1024] = {0};
    int readLen = 0;
    int count = 0;
    int oldLen = readBuffer.size();
    do{
        readLen = readBufferMaxSize - readBuffer.size();
        if(readLen < 1024){
            len = ::read(fileFd, readData, readLen);
            readBuffer.append(readData, len);
        }
        else{
            len = ::read(fileFd, readData, 1024);
            readBuffer.append(readData, len);
        }
        count++;
    } while(len == 1024 && readLen != 0);  // readData low than 10k and read data low than 1024 loop break;
    ssize_t returnlen = readBuffer.size() - oldLen;
    return returnlen;
}
