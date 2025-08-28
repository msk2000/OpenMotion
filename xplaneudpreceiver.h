// xplaneudpreceiver.h
// -----------------------------------------------------------------------------
// A tiny Qt-based UDP listener for X‑Plane "DATA\0" packets that extracts
//   - Euler orientation (deg): roll, pitch, yaw          [index 17]
//   - Body rates (rad/s): p, q, r                        [index 16, deg/s -> rad/s]
//   - Linear accelerations (m/s^2): ax, ay, az           [index 4, g -> m/s^2]
//
// Usage:
//   auto rx = new XPlaneUdpReceiver(this);
//   rx->start(49005); // bind to the port you set as X‑Plane's destination
//   connect(rx, &XPlaneUdpReceiver::sampleReady, this, [](const MotionSample& s){ ... });
//
// Notes:
//   * This class is header-only for convenience.
//   * Uses QUdpSocket with ShareAddress/ReuseAddressHint.
//   * Parses X‑Plane DATA groups: 5-byte header "DATA\0", then N groups of 36 bytes:
//       int32 index; float vals[8];
//   * All floats are little-endian on typical PCs. We memcpy into native floats.
// -----------------------------------------------------------------------------

#ifndef XPLANEUDPRECEIVER_H
#define XPLANEUDPRECEIVER_H


#pragma once

#include "qdebug.h"
#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QtGlobal>    // for quint8/uchar
#include <cstring>     // std::memcpy
#include <cmath>       // M_PI


// ---------------------------- Configuration ----------------------------------
// X‑Plane DATA indices we care about (can be customized).
//  17: pitch, roll, heading (all deg)
//  16: p, q, r angular rates (deg/s)
//   4: body-axis accelerations in g
static constexpr int XP_IDX_EULER_DEG   = 17;
static constexpr int XP_IDX_RATES_DEGPS = 16;
static constexpr int XP_IDX_ACCEL_G     = 4;

// Constant for unit conversion
static constexpr float DEG2RAD_F = float(M_PI / 180.0);
static constexpr float G_TO_MS2  = 9.81f;

// ----------------------------- Data struct -----------------------------------
struct MotionSample
{
    // Orientation (deg)
    float roll_deg  = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg   = 0.0f;

    // Body rates (rad/s)
    float p = 0.0f;
    float q = 0.0f;
    float r = 0.0f;

    // Linear accelerations (m/s^2), body axes
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
};

// ----------------------------- Receiver class --------------------------------
class XPlaneUdpReceiver : public QObject
{
    Q_OBJECT
public:
    explicit XPlaneUdpReceiver(QObject* parent = nullptr)
        : QObject(parent)
    {}

    ~XPlaneUdpReceiver() override
    {
        stop(); // ensure socket is cleaned up
    }

    // Bind the UDP socket to the given port (AnyIPv4). Returns true on success.
    // Call with the same port configured as X‑Plane's "send to" destination.
    bool start(quint16 port,
               const QHostAddress& bindAddr = QHostAddress::AnyIPv4)
    {
        // Create socket if first time
        if (!sock_)
        {
            sock_ = new QUdpSocket(this);
        } else
        {
            // If already bound to the same port, nothing to do.
            if (sock_->state() == QAbstractSocket::BoundState && boundPort_ == port) {
                return true;
            }
            // Otherwise, unbind first
            stop();
            sock_ = new QUdpSocket(this);
        }

        // ShareAddress/ReuseAddressHint ~ SO_REUSEADDR; friendlier restarts.
        const bool ok = sock_->bind(bindAddr, port,
                                    QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
        if (!ok)
        {
            qWarning() << "[XPlaneUdpReceiver] Failed to bind UDP port"
                       << port << ":" << sock_->errorString();
            delete sock_;
            sock_ = nullptr;
            boundPort_ = 0;
            return false;
        }

        boundPort_ = port;

        // Wire up readyRead signal to our slot that parses DATA packets
        connect(sock_, &QUdpSocket::readyRead,
                this,  &XPlaneUdpReceiver::onReadyRead);

        qDebug() << "[UDP] Bound on port" << port;

        return true;
    }

    // Unbind and destroy the socket (safe to call multiple times).
    void stop()
    {
        if (sock_)
        {
            sock_->disconnect(this);
            sock_->close();
            sock_->deleteLater();
            sock_ = nullptr;
            boundPort_ = 0;
        }
    }

    // Returns the currently bound port (or 0 if not bound).
    quint16 boundPort() const noexcept { return boundPort_; }

signals:
    // Emitted on each DATA packet (or set of groups in a packet) with the most recent values.
    // Note: Fields only update when their respective indices appear in the packet.
    void sampleReady(const MotionSample& s);

private slots:
    // Socket ready to read , consume all pending datagrams.
    void onReadyRead()
    {

        qDebug() << "[UDP] pending datagrams:" << sock_->pendingDatagramSize();

        if (!sock_) return;

        while (sock_->hasPendingDatagrams())
        {
            QByteArray datagram;
            datagram.resize(int(sock_->pendingDatagramSize()));
            const qint64 n = sock_->readDatagram(datagram.data(), datagram.size());
            if (n <= 0) continue;

            // DATA packets start with "DATA\0" (5 bytes)
            qDebug() << "[UDP] got packet size" << datagram.size()
                     << "first 5 bytes:" << QByteArray(datagram.constData(), 5);

            if (datagram.size() < 9) continue; // too short to contain any groups
            const quint8* b = reinterpret_cast<const quint8*>(datagram.constData());
            // Accept DATA\0, DATA@, etc.
            if (!(b[0]=='D' && b[1]=='A' && b[2]=='T' && b[3]=='A')) {
                continue;
            }



            // Parse into our MotionSample and emit
            MotionSample out = last_; // start from last values so absent groups don't zero fields
            parseData(datagram, out);
            last_ = out;
            emit sampleReady(out);
        }
    }

private:
    // Copy 4 bytes into a float (alignment-safe; assumes little-endian sender & host)
    static inline float loadFloatLE(const quint8* p)
    {
        float v;
        std::memcpy(&v, p, sizeof(float));
        return v;
    }

    // Parse the "DATA\0" payload into a MotionSample.
    // Layout: starting at offset 5, there are groups of 36 bytes:
    //   int32 index; float vals[8];
    void parseData(const QByteArray& payload, MotionSample& out)
    {
        const quint8* b = reinterpret_cast<const quint8*>(payload.constData());
        const int n = payload.size();

        // Walk over each 36-byte group
        for (int off = 5; off + 36 <= n; off += 36)
        {
            // Read index
            int idx = 0;
            std::memcpy(&idx, b + off, sizeof(int)); // little-endian on typical PC

            // Read 8 floats after the index
            float vals[8];
            std::memcpy(vals, b + off + 4, 8 * sizeof(float));

            qDebug() << "[UDP] group idx" << idx << "vals0..2"
                     << vals[0] << vals[1] << vals[2];


            switch (idx)
            {
            case XP_IDX_EULER_DEG:
                // X‑Plane index 17 order: pitch, roll, heading (all degrees)
                out.pitch_deg = vals[0];
                out.roll_deg  = vals[1];
                out.yaw_deg   = vals[2];
                break;

            case XP_IDX_RATES_DEGPS:
                // Convert deg/s to rad/s for p, q, r
                out.p = vals[0] * DEG2RAD_F;
                out.q = vals[1] * DEG2RAD_F;
                out.r = vals[2] * DEG2RAD_F;
                break;

            case XP_IDX_ACCEL_G:
                // Convert g to m/s^2 for ax, ay, az
                out.ax = vals[0] * G_TO_MS2;
                out.ay = vals[1] * G_TO_MS2;
                out.az = vals[2] * G_TO_MS2;
                break;

            default:
                // Ignore other groups
                break;
            }
        }
    }


    QUdpSocket* sock_   = nullptr;   // owned by this via parent
    quint16     boundPort_ = 0;      // 0 if not bound
    MotionSample last_{};            // last good values (used to keep fields stable)
};

#endif // XPLANEUDPRECEIVER_H
