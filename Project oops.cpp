/*
 * ============================================================================
 *  PROJECT  : Autonomous Landslide & Flash-Flood Early Warning Grid (ALF-EWG)
 *  SUBJECT  : Object-Oriented Programming Using C++   (Paper Code: CIC-211)
 *  INSTITUTE: Maharaja Agrasen Institute of Technology
 *  STANDARD : C++11  (also compiles as C++14 / C++17)
 *
 *  BUILD    : g++ -std=c++11 -Wall -Wextra -o alf_ewg alf_ewg.cpp
 *  RUN      : ./alf_ewg          (Windows: alf_ewg.exe)
 *
 *  HOW IT WORKS
 *  A small network of solar-powered telemetry nodes is "swept" in cycles while
 *  a simulated storm grows stronger. Every node reports sensor data, the grid
 *  turns it into a hazard index (0-100) and an alert level, sirens are
 *  triggered per region, evacuation tasks are ranked by priority and every
 *  serious incident is written to a binary log file on disk.
 *  All sensor data is SIMULATED inside the program (no input needed).
 *
 *  WHERE EACH SYLLABUS REQUIREMENT IS DEMONSTRATED  (search for the [tag])
 *  ---------------------------------------------------------------------------
 *  UNIT I
 *   [U1.1] Namespaces DisasterCore::Sensors and DisasterCore::Alerts
 *   [U1.2] Inline functions for real-time risk indexing
 *   [U1.3] Overloaded computeHazardIndex()
 *   [U1.4] Call by reference / return by reference (coordinates, accumulation)
 *   [U1.5] Friend class TelemetryAuditor reads private raw metrics (no getters)
 *  UNIT II
 *   [U2.1] SensorStation, EarlyWarningSiren, TelemetryBuffer: data abstraction
 *          and const correctness
 *   [U2.2] Static data members + static member functions
 *   [U2.3] Dynamic buffer: parameterised / dynamic / deep-copy constructors,
 *          explicit destructor
 *   [U2.4] Operators: binary +, relational >, stream << and >>
 *  UNIT III
 *   [U3.1] Abstract base class BaseSensorNode (pure virtual functions)
 *   [U3.2] Diamond problem solved with virtual base classes
 *   [U3.3] Containership: PowerSubsystem object inside BaseSensorNode
 *   [U3.4] std::vector<BaseSensorNode*> and run-time (vtable) dispatch
 *  UNIT IV
 *   [U4.1] Custom exceptions derived from std::exception with what()
 *   [U4.2] Binary std::fstream log with seekp()/seekg() and rollback
 *   [U4.3] STL vector/map, sort, for_each, functors, custom manipulators
 * ============================================================================
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/* ============================================================================
 *  COMMON HELPERS AND CUSTOM EXCEPTIONS
 * ========================================================================== */
namespace DisasterCore {

// Turn a number into text with a fixed number of decimals (used for printing).
inline std::string fmt(double value, int decimals = 1) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(decimals) << value;
    return oss.str();
}

inline std::string toStr(int value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

// [U4.1] Custom exception 1: a sensor node can no longer deliver data.
class SensorFailureException : public std::exception {
private:
    std::string message_;
public:
    SensorFailureException(int nodeId, const std::string& reason) {
        std::ostringstream oss;
        oss << "SensorFailureException: node " << nodeId << " failed - " << reason;
        message_ = oss.str();
    }
    const char* what() const noexcept override { return message_.c_str(); }
};

// [U4.1] Custom exception 2: received data does not match its checksum.
class TelemetryChecksumMismatch : public std::exception {
private:
    std::string message_;
public:
    TelemetryChecksumMismatch(const std::string& source, unsigned int expected,
                              unsigned int received) {
        std::ostringstream oss;
        oss << "TelemetryChecksumMismatch: " << source << " - expected 0x" << std::hex
            << expected << ", got 0x" << received;
        message_ = oss.str();
    }
    const char* what() const noexcept override { return message_.c_str(); }
};

// [U4.1] Custom exception 3: a value is beyond the range the system accepts.
class ThresholdOverflowException : public std::exception {
private:
    std::string message_;
public:
    ThresholdOverflowException(const std::string& source, double value, double limit) {
        std::ostringstream oss;
        oss << "ThresholdOverflowException: " << source << " reported " << fmt(value, 1)
            << " which exceeds the limit of " << fmt(limit, 1);
        message_ = oss.str();
    }
    const char* what() const noexcept override { return message_.c_str(); }
};

} // namespace DisasterCore

/* ============================================================================
 *  NAMESPACE  DisasterCore::Alerts   (part 1 - alert levels)
 * ========================================================================== */
namespace DisasterCore {
namespace Alerts {   // [U1.1]

enum AlertLevel { ALERT_NORMAL = 0, ALERT_WATCH = 1, ALERT_WARNING = 2, ALERT_EMERGENCY = 3 };

inline const char* levelName(AlertLevel level) {
    switch (level) {
        case ALERT_NORMAL:    return "NORMAL";
        case ALERT_WATCH:     return "WATCH";
        case ALERT_WARNING:   return "WARNING";
        case ALERT_EMERGENCY: return "EMERGENCY";
    }
    return "UNKNOWN";
}

// [U1.2] Inline function: hazard index (0-100) -> alert level.
inline AlertLevel levelFromHazard(double hazard) {
    if (hazard >= 75.0) return ALERT_EMERGENCY;
    if (hazard >= 50.0) return ALERT_WARNING;
    if (hazard >= 25.0) return ALERT_WATCH;
    return ALERT_NORMAL;
}

} // namespace Alerts
} // namespace DisasterCore

/* ============================================================================
 *  NAMESPACE  DisasterCore::Sensors
 * ========================================================================== */
namespace DisasterCore {
namespace Sensors {   // [U1.1]

// ---------------------------------------------------------------- data types
struct GeoCoordinate {
    double latitude;
    double longitude;
    GeoCoordinate(double lat = 0.0, double lon = 0.0) : latitude(lat), longitude(lon) {}
};

// One complete set of readings from a site.
struct TelemetryReading {
    double rainfallRate;    // mm per hour
    double slopeAngle;      // degrees (terrain, fixed for a site)
    double soilSaturation;  // percent
    double vibrationFreq;   // Hz (seismic geophone)
    double tiltAngle;       // degrees (acoustic tiltmeter)
    double riverLevel;      // metres (sonar gauge), 0 = no river gauge at this site
    TelemetryReading()
        : rainfallRate(0.0), slopeAngle(0.0), soilSaturation(0.0),
          vibrationFreq(0.0), tiltAngle(0.0), riverLevel(0.0) {}
};

inline TelemetryReading makeBaseline(double rain, double slope, double soil,
                                     double vib, double tilt, double river) {
    TelemetryReading r;
    r.rainfallRate = rain;  r.slopeAngle = slope;  r.soilSaturation = soil;
    r.vibrationFreq = vib;  r.tiltAngle = tilt;    r.riverLevel = river;
    return r;
}

// Running totals for one node (filled by accumulate()).
struct TelemetryState {
    int    samples;
    double rainfallSum;
    double peakHazard;
    double peakVibration;
    TelemetryState() : samples(0), rainfallSum(0.0), peakHazard(0.0), peakVibration(0.0) {}
};

// ------------------------------------------- [U1.2] inline risk-index maths
inline double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
inline double normalise(double value, double maxValue) { return clamp01(value / maxValue); }

// [U1.3] OVERLOADED computeHazardIndex(): same name, different telemetry
// combinations. The compiler picks the version from the number of arguments.

// (a) rainfall rate + slope angle  -> rain drives the risk, steeper slope amplifies it
inline double computeHazardIndex(double rainfallRate, double slopeAngle) {
    return 100.0 * normalise(rainfallRate, 100.0) * (0.4 + 0.6 * normalise(slopeAngle, 60.0));
}

// (b) soil saturation + vibration frequency + tilt angle
//     (saturation below 40 % adds no risk)
inline double computeHazardIndex(double soilSaturation, double vibrationFreq, double tiltAngle) {
    return 100.0 * (0.5 * normalise(soilSaturation - 40.0, 60.0)
                  + 0.3 * normalise(vibrationFreq, 40.0)
                  + 0.2 * normalise(tiltAngle, 10.0));
}

// (c) river level only (2 m = normal flow, 8 m = flood level)
inline double computeHazardIndex(double riverLevel) {
    return 100.0 * normalise(riverLevel - 2.0, 6.0);
}

// Worst-case hazard of a complete reading (uses all three overloads).
inline double overallHazard(const TelemetryReading& r) {
    double a = computeHazardIndex(r.rainfallRate, r.slopeAngle);
    double b = computeHazardIndex(r.soilSaturation, r.vibrationFreq, r.tiltAngle);
    double c = computeHazardIndex(r.riverLevel);
    return std::max(a, std::max(b, c));
}

// ------------------------------- [U1.4] call by reference / return by reference
// The coordinate is passed BY REFERENCE (no copy is made) and the same object is
// RETURNED BY REFERENCE, so calls can be chained:  snapToGrid(shiftCoordinate(c, ..), ..)
inline GeoCoordinate& shiftCoordinate(GeoCoordinate& c, double dLat, double dLon) {
    c.latitude += dLat;
    c.longitude += dLon;
    return c;
}

inline GeoCoordinate& snapToGrid(GeoCoordinate& c, double cell) {
    c.latitude  = std::floor(c.latitude  / cell + 0.5) * cell;
    c.longitude = std::floor(c.longitude / cell + 0.5) * cell;
    return c;
}

// Returns a REFERENCE to a member, so the caller can read it or even assign to it.
inline double& peakHazardOf(TelemetryState& s) { return s.peakHazard; }

// Telemetry state accumulation: updates the caller's object directly.
inline TelemetryState& accumulate(TelemetryState& s, const TelemetryReading& r) {
    double  hazard = overallHazard(r);
    double& peak   = peakHazardOf(s);          // reference variable = alias of s.peakHazard
    if (hazard > peak) peak = hazard;
    if (r.vibrationFreq > s.peakVibration) s.peakVibration = r.vibrationFreq;
    s.rainfallSum += r.rainfallRate;
    ++s.samples;
    return s;
}

// ------------------------------------------------ simulation environment
// [U2.2] Only static members: one shared "weather" for the whole network.
class SimEnvironment {
public:
    static double stormIntensity;     // 0.0 calm ... 1.8 extreme cloudburst
    static bool   linkInterference;   // true = radio noise corrupts LoRa packets
    static bool   traceLifecycle;     // true = print constructor/destructor messages
    static double noise() {           // tiny repeatable pseudo-random value in [-0.05, +0.05]
        seed_ = seed_ * 1664525u + 1013904223u;       // linear congruential generator
        double unit = (seed_ >> 8) / 16777216.0;      // 0.0 <= unit < 1.0
        return (unit - 0.5) * 0.10;
    }
private:
    static unsigned int seed_;
};
double       SimEnvironment::stormIntensity   = 0.0;
bool         SimEnvironment::linkInterference = false;
bool         SimEnvironment::traceLifecycle   = false;
unsigned int SimEnvironment::seed_            = 2026u;

inline void traceLife(const char* text) {
    if (SimEnvironment::traceLifecycle) std::cout << "    " << text << "\n";
}

// ------------------------------------------------- [U3.3] contained object
// PowerSubsystem is an independent class. BaseSensorNode HAS-A PowerSubsystem
// (containership / aggregation). It is not a parent class.
class PowerSubsystem {
private:
    double batteryPct_;
public:
    explicit PowerSubsystem(double pct = 100.0) : batteryPct_(pct) {}
    void consume(double pct)  { batteryPct_ -= pct; if (batteryPct_ < 0.0)   batteryPct_ = 0.0; }
    void recharge(double pct) { batteryPct_ += pct; if (batteryPct_ > 100.0) batteryPct_ = 100.0; }
    bool   isDepleted() const { return batteryPct_ <= 5.0; }
    double level() const      { return batteryPct_; }
};

// -------------------------------------------- [U2.3] dynamic memory buffer
// Keeps the most recent hazard values of a station in a circular array that is
// allocated on the heap. Because the class owns raw memory it needs its own
// constructors, copy constructor, assignment operator and destructor.
class TelemetryBuffer {
private:
    char*   label_;       // heap memory #1
    double* samples_;     // heap memory #2
    int     capacity_;
    int     count_;
    int     head_;        // where the next value will be written
public:
    // Parameterised constructor + DYNAMIC constructor: it obtains memory with
    // new at the moment the object is created. Because of the default
    // arguments it also serves as the default constructor.
    explicit TelemetryBuffer(const char* label = "unnamed", int capacity = 4)
        : label_(0), samples_(0), capacity_(capacity < 1 ? 1 : capacity), count_(0), head_(0) {
        label_ = new char[std::strlen(label) + 1];
        std::strcpy(label_, label);
        samples_ = new double[capacity_];
        for (int i = 0; i < capacity_; ++i) samples_[i] = 0.0;
    }

    // DEEP COPY constructor: allocates NEW memory and copies the contents, so the
    // copy and the original never share (or double-delete) the same array.
    TelemetryBuffer(const TelemetryBuffer& other)
        : label_(0), samples_(0), capacity_(other.capacity_), count_(other.count_),
          head_(other.head_) {
        label_ = new char[std::strlen(other.label_) + 1];
        std::strcpy(label_, other.label_);
        samples_ = new double[capacity_];
        for (int i = 0; i < capacity_; ++i) samples_[i] = other.samples_[i];
    }

    // Copy assignment (rule of three): same deep-copy idea.
    TelemetryBuffer& operator=(const TelemetryBuffer& other) {
        if (this == &other) return *this;
        char*   newLabel   = new char[std::strlen(other.label_) + 1];
        std::strcpy(newLabel, other.label_);
        double* newSamples = new double[other.capacity_];
        for (int i = 0; i < other.capacity_; ++i) newSamples[i] = other.samples_[i];
        delete[] label_;
        delete[] samples_;
        label_ = newLabel;  samples_ = newSamples;
        capacity_ = other.capacity_;  count_ = other.count_;  head_ = other.head_;
        return *this;
    }

    // Explicit destructor: gives the heap memory back (no leaks).
    ~TelemetryBuffer() {
        delete[] label_;
        delete[] samples_;
    }

    void push(double value) {
        samples_[head_] = value;
        head_ = (head_ + 1) % capacity_;
        if (count_ < capacity_) ++count_;
    }
    // [U2.1] const member functions: they promise not to change the object.
    double average() const {
        if (count_ == 0) return 0.0;
        double sum = 0.0;
        for (int i = 0; i < count_; ++i) sum += samples_[i];
        return sum / count_;
    }
    int         size() const     { return count_; }
    int         capacity() const { return capacity_; }
    const char* label() const    { return label_; }
    const void* dataAddress() const { return static_cast<const void*>(samples_); }
};

// ---------------------------------------------- [U3.1] abstract base class
class BaseSensorNode {
protected:
    int           id_;
    std::string   name_;
    GeoCoordinate location_;
    PowerSubsystem power_;      // [U3.3] containership: BaseSensorNode HAS-A PowerSubsystem
public:
    BaseSensorNode(int id, const std::string& name, const GeoCoordinate& loc, double batteryPct)
        : id_(id), name_(name), location_(loc), power_(batteryPct) {}

    // Virtual destructor: deleting a derived object through a BaseSensorNode*
    // must run the destructor of the real (derived) class.
    virtual ~BaseSensorNode() {}

    // Pure virtual functions: every concrete node MUST provide them.
    virtual TelemetryReading     readTelemetry() = 0;
    virtual Alerts::AlertLevel   evaluateThreshold() const = 0;

    // Ordinary virtual function with a default body (can be overridden).
    virtual std::string nodeType() const { return "GenericNode"; }

    int                  id() const            { return id_; }
    const std::string&   name() const          { return name_; }
    const GeoCoordinate& position() const      { return location_; }
    double               batteryLevel() const  { return power_.level(); }
};

// ------------------------------------------- [U3.2] the diamond hierarchy
//
//                  BaseSensorNode            (abstract)
//                         |
//             DisasterTelemetryDevice        (ROOT of the diamond)
//                /                \           both inherit it "virtual"
//     SolarPoweredNode     WirelessLoRaTransceiver
//                \                /
//             AutonomousHybridProbe          (derived leaf)
//
// Without "virtual", the probe would contain TWO copies of
// DisasterTelemetryDevice (and BaseSensorNode) and any use of id_ or name_
// would be ambiguous. With "virtual" there is exactly ONE shared copy.

class DisasterTelemetryDevice : public BaseSensorNode {
protected:
    std::string serial_;
    int         cyclesOnline_;
public:
    DisasterTelemetryDevice(int id, const std::string& name, const GeoCoordinate& loc,
                            double battery)
        : BaseSensorNode(id, name, loc, battery), serial_("DTD-" + toStr(id)), cyclesOnline_(0) {
        traceLife("[ctor] DisasterTelemetryDevice   <- virtual base, built only ONCE");
    }
    virtual ~DisasterTelemetryDevice() { traceLife("[dtor] DisasterTelemetryDevice"); }

    const std::string& serial() const { return serial_; }
    void tick()                       { ++cyclesOnline_; }
    int  uptimeCycles() const         { return cyclesOnline_; }
};

class SolarPoweredNode : virtual public DisasterTelemetryDevice {
protected:
    double panelWatts_;
public:
    // Every class on the path must name the virtual base in its initialiser
    // list, but only the MOST DERIVED class's call is actually executed.
    SolarPoweredNode(int id, const std::string& name, const GeoCoordinate& loc,
                     double battery, double panelWatts)
        : DisasterTelemetryDevice(id, name, loc, battery), panelWatts_(panelWatts) {
        traceLife("[ctor] SolarPoweredNode");
    }
    virtual ~SolarPoweredNode() { traceLife("[dtor] SolarPoweredNode"); }

    // Recharges the CONTAINED PowerSubsystem. sunlight: 0.0 (dark) .. 1.0 (clear sky)
    void harvestSolar(double sunlight) { power_.recharge(panelWatts_ * sunlight * 0.2); }
};

class WirelessLoRaTransceiver : virtual public DisasterTelemetryDevice {
protected:
    int channel_;
    int packetsSent_;
public:
    WirelessLoRaTransceiver(int id, const std::string& name, const GeoCoordinate& loc,
                            double battery, int channel)
        : DisasterTelemetryDevice(id, name, loc, battery), channel_(channel), packetsSent_(0) {
        traceLife("[ctor] WirelessLoRaTransceiver");
    }
    virtual ~WirelessLoRaTransceiver() { traceLife("[dtor] WirelessLoRaTransceiver"); }

    static unsigned int packetChecksum(const TelemetryReading& r) {
        unsigned int sum = 7u;
        sum = sum * 131u + static_cast<unsigned int>(r.soilSaturation * 100.0);
        sum = sum * 131u + static_cast<unsigned int>(r.vibrationFreq * 100.0);
        sum = sum * 131u + static_cast<unsigned int>(r.tiltAngle * 100.0);
        sum = sum * 131u + static_cast<unsigned int>(r.riverLevel * 100.0);
        return sum;
    }

    // Simulated radio link. The receiver recomputes the checksum of what it
    // actually received; radio noise (linkInterference) makes it differ.
    void transmitPacket(const TelemetryReading& r) {
        unsigned int sentChecksum = packetChecksum(r);
        TelemetryReading onAir = r;                         // what really arrives
        if (SimEnvironment::linkInterference) onAir.soilSaturation += 7.3;   // bit errors
        ++packetsSent_;
        unsigned int recomputed = packetChecksum(onAir);
        if (recomputed != sentChecksum)
            throw TelemetryChecksumMismatch("LoRa packet from " + name_, sentChecksum, recomputed);
    }
};

class AutonomousHybridProbe : public SolarPoweredNode, public WirelessLoRaTransceiver {
private:
    TelemetryReading last_;
public:
    // The MOST DERIVED class initialises the virtual base itself.
    AutonomousHybridProbe(int id, const std::string& name, const GeoCoordinate& loc,
                          double battery, double panelWatts, int channel)
        : DisasterTelemetryDevice(id, name, loc, battery),
          SolarPoweredNode(id, name, loc, battery, panelWatts),
          WirelessLoRaTransceiver(id, name, loc, battery, channel) {
        traceLife("[ctor] AutonomousHybridProbe");
    }
    virtual ~AutonomousHybridProbe() { traceLife("[dtor] AutonomousHybridProbe"); }

    // [U3.4] overrides of the pure virtual functions of BaseSensorNode
    TelemetryReading readTelemetry() override {
        if (power_.isDepleted()) throw SensorFailureException(id_, "probe battery depleted");
        power_.consume(3.0);
        double s = SimEnvironment::stormIntensity;
        harvestSolar(1.0 - clamp01(s));                      // storm clouds block the sun
        last_.soilSaturation = std::min(100.0, 45.0 + 35.0 * s);
        last_.vibrationFreq  = 6.0 * (0.5 + s) * (1.0 + SimEnvironment::noise());
        last_.tiltAngle      = 1.5 * (0.5 + s);
        // The probe drifts a little each sweep; coordinates are fixed by reference.
        snapToGrid(shiftCoordinate(location_, 0.0003, -0.0002), 0.0001);
        tick();
        transmitPacket(last_);                               // may throw TelemetryChecksumMismatch
        return last_;
    }
    Alerts::AlertLevel evaluateThreshold() const override {
        return Alerts::levelFromHazard(overallHazard(last_));
    }
    std::string nodeType() const override { return "HybridProbe"; }
};

// ----------------------------------------------------------- SensorStation
class TelemetryAuditor;   // forward declaration (needed for the friend line below)

// A fixed ground station: a cluster of sensors (rain gauge, soil probe,
// geophone, tiltmeter and - where there is a river - a sonar gauge).
// [U2.1] Data abstraction: all data is private; the outside world uses the
// public interface only.
class SensorStation : public BaseSensorNode {
private:
    std::string      zone_;
    TelemetryReading baseline_;   // the site's normal conditions (used by the simulator)
    TelemetryReading raw_;        // latest RAW metrics: private and deliberately WITHOUT getters
    TelemetryBuffer  history_;    // [U2.3] heap buffer with the last 5 hazard values

    // [U2.2] static data members: shared by ALL stations
    static int                 activeStations_;
    static Alerts::AlertLevel  globalAlert_;

    static const double MAX_RAINFALL_RANGE;   // the rain gauge cannot measure above this

    // [U1.5] A friend class may look at private data.
    friend class TelemetryAuditor;
    // [U2.4] Stream operators need access to the private data, so they are friends too.
    friend std::ostream& operator<<(std::ostream& os, const SensorStation& s);
    friend std::istream& operator>>(std::istream& is, SensorStation& s);

public:
    SensorStation()
        : BaseSensorNode(0, "Unnamed", GeoCoordinate(), 100.0), zone_("Unassigned"),
          history_("unnamed", 5) {
        ++activeStations_;
    }

    SensorStation(int id, const std::string& name, const std::string& zone,
                  const GeoCoordinate& loc, double battery, const TelemetryReading& baseline)
        : BaseSensorNode(id, name, loc, battery), zone_(zone), baseline_(baseline),
          history_(name.c_str(), 5) {
        ++activeStations_;
    }

    // Copy constructor: base part and members are copied (history_ is deep-copied
    // by TelemetryBuffer's own copy constructor) and the station count is updated.
    SensorStation(const SensorStation& other)
        : BaseSensorNode(other), zone_(other.zone_), baseline_(other.baseline_),
          raw_(other.raw_), history_(other.history_) {
        ++activeStations_;
    }

    virtual ~SensorStation() { --activeStations_; }

    // ------------------------------------------------ [U2.2] static functions
    static int  activeStations()               { return activeStations_; }
    static Alerts::AlertLevel globalAlert()    { return globalAlert_; }
    static void raiseGlobalAlert(Alerts::AlertLevel level) {
        if (level > globalAlert_) globalAlert_ = level;
    }
    static void resetGlobalAlert()             { globalAlert_ = Alerts::ALERT_NORMAL; }

    // ------------------------------------------- [U3.4] virtual overrides
    TelemetryReading readTelemetry() override {
        if (power_.isDepleted())
            throw SensorFailureException(id_, "battery depleted, station cannot sample");
        power_.consume(4.0);
        const double s = SimEnvironment::stormIntensity;
        raw_.rainfallRate   = baseline_.rainfallRate * s * (1.0 + SimEnvironment::noise());
        raw_.slopeAngle     = baseline_.slopeAngle;
        raw_.soilSaturation = std::min(100.0, baseline_.soilSaturation + 35.0 * s);
        raw_.vibrationFreq  = baseline_.vibrationFreq * (0.5 + s) * (1.0 + SimEnvironment::noise());
        raw_.tiltAngle      = baseline_.tiltAngle * (0.5 + s);
        raw_.riverLevel     = baseline_.riverLevel * (1.0 + 0.9 * s);
        history_.push(hazardIndex());
        return raw_;
    }

    Alerts::AlertLevel evaluateThreshold() const override {
        if (raw_.rainfallRate > MAX_RAINFALL_RANGE) {
            raiseGlobalAlert(Alerts::ALERT_EMERGENCY);      // fail-safe: out-of-range is itself an alarm
            throw ThresholdOverflowException(name_ + " rain gauge", raw_.rainfallRate,
                                             MAX_RAINFALL_RANGE);
        }
        Alerts::AlertLevel level = Alerts::levelFromHazard(hazardIndex());
        raiseGlobalAlert(level);    // a const function may still update STATIC data
        return level;
    }

    std::string nodeType() const override { return "Station"; }

    // ---------------------------------------------- public (const) interface
    const std::string& zone() const         { return zone_; }
    double hazardIndex() const              { return overallHazard(raw_); }
    double recentAverageHazard() const      { return history_.average(); }

    // [U2.4] relational operator: the station with the HIGHER hazard is "greater".
    bool operator>(const SensorStation& other) const { return hazardIndex() > other.hazardIndex(); }
};

int                SensorStation::activeStations_ = 0;
Alerts::AlertLevel SensorStation::globalAlert_    = Alerts::ALERT_NORMAL;
const double       SensorStation::MAX_RAINFALL_RANGE = 150.0;

// [U2.4] Stream insertion: one line of text, fields separated by spaces,
// so the SAME format can be sent to the console or to a file and read back.
std::ostream& operator<<(std::ostream& os, const SensorStation& s) {
    std::ios_base::fmtflags oldFlags = os.flags();
    std::streamsize oldPrecision = os.precision();
    os << s.id_ << ' ' << s.name_ << ' ' << s.zone_ << ' '
       << std::fixed << std::setprecision(4) << s.location_.latitude << ' '
       << s.location_.longitude << ' '
       << std::setprecision(1) << s.power_.level() << ' '
       << std::setprecision(3) << s.raw_.rainfallRate << ' ' << s.raw_.slopeAngle << ' '
       << s.raw_.soilSaturation << ' ' << s.raw_.vibrationFreq << ' ' << s.raw_.tiltAngle << ' '
       << s.raw_.riverLevel;
    os.flags(oldFlags);
    os.precision(oldPrecision);
    return os;
}

// [U2.4] Stream extraction: reads the same format back into a station.
std::istream& operator>>(std::istream& is, SensorStation& s) {
    double battery = 0.0;
    is >> s.id_ >> s.name_ >> s.zone_ >> s.location_.latitude >> s.location_.longitude >> battery
       >> s.raw_.rainfallRate >> s.raw_.slopeAngle >> s.raw_.soilSaturation
       >> s.raw_.vibrationFreq >> s.raw_.tiltAngle >> s.raw_.riverLevel;
    s.power_ = PowerSubsystem(battery);
    return is;
}

// [U1.5] FRIEND CLASS: reads the private raw metrics directly. SensorStation has
// no getters for them, so only friends can inspect them.
class TelemetryAuditor {
public:
    static bool audit(const SensorStation& s, std::ostream& os) {
        const TelemetryReading& r = s.raw_;
        bool plausible = r.soilSaturation >= 0.0 && r.soilSaturation <= 100.0 &&
                         r.vibrationFreq >= 0.0 && r.tiltAngle >= 0.0 && r.rainfallRate >= 0.0;
        os << "  audit " << s.name_ << ": rain=" << fmt(r.rainfallRate) << " mm/h  soil="
           << fmt(r.soilSaturation) << " %  vib=" << fmt(r.vibrationFreq) << " Hz  tilt="
           << fmt(r.tiltAngle) << " deg  river=" << fmt(r.riverLevel, 2) << " m  -> "
           << (plausible ? "plausible" : "SUSPICIOUS") << "\n";
        return plausible;
    }
};

// ------------------------------------------------------------ RegionalZone
// A "zone vector": one hazard value for every station that belongs to the zone.
class RegionalZone {
private:
    std::string              name_;
    std::vector<std::string> members_;
    std::vector<double>      hazards_;
public:
    explicit RegionalZone(const std::string& name = "Unassigned") : name_(name) {}

    void add(const SensorStation& s) {
        members_.push_back(s.name());
        hazards_.push_back(s.hazardIndex());
    }

    // [U2.4] zone + station -> a new, bigger zone
    RegionalZone operator+(const SensorStation& s) const {
        RegionalZone result(*this);
        result.add(s);
        return result;
    }

    // [U2.4] zones can be ranked too: the more dangerous zone is "greater"
    bool operator>(const RegionalZone& other) const { return peakHazard() > other.peakHazard(); }

    double peakHazard() const {
        double peak = 0.0;
        for (std::size_t i = 0; i < hazards_.size(); ++i) peak = std::max(peak, hazards_[i]);
        return peak;
    }
    double meanHazard() const {
        if (hazards_.empty()) return 0.0;
        double sum = 0.0;
        for (std::size_t i = 0; i < hazards_.size(); ++i) sum += hazards_[i];
        return sum / hazards_.size();
    }
    int  stationCount() const { return static_cast<int>(hazards_.size()); }
    const std::string& name() const { return name_; }
    // The worst station decides the alert level of the whole zone.
    Alerts::AlertLevel level() const { return Alerts::levelFromHazard(peakHazard()); }
};

// [U2.4] BINARY + : station + station -> RegionalZone
RegionalZone operator+(const SensorStation& a, const SensorStation& b) {
    RegionalZone zone(a.zone() == b.zone() ? a.zone() : a.zone() + "+" + b.zone());
    zone.add(a);
    zone.add(b);
    return zone;
}

} // namespace Sensors
} // namespace DisasterCore

/* ============================================================================
 *  NAMESPACE  DisasterCore::Alerts   (part 2 - sirens, dispatch, log file)
 * ========================================================================== */
namespace DisasterCore {
namespace Alerts {   // [U1.1]

// -------------------------------------------------------------- the siren
// [U2.1] Data abstraction: private state, controlled through member functions;
// the "query" functions are const.
class EarlyWarningSiren {
private:
    std::string zoneName_;
    AlertLevel  level_;
    int         volumeDb_;
public:
    explicit EarlyWarningSiren(const std::string& zoneName = "Unassigned")
        : zoneName_(zoneName), level_(ALERT_NORMAL), volumeDb_(0) {}

    void trigger(AlertLevel level) {
        level_ = level;
        if (level == ALERT_EMERGENCY)    volumeDb_ = 110;
        else if (level == ALERT_WARNING) volumeDb_ = 95;
        else                             volumeDb_ = 0;       // WATCH and NORMAL: no siren
    }
    void silence() { level_ = ALERT_NORMAL; volumeDb_ = 0; }

    bool        isSounding() const { return volumeDb_ > 0; }
    int         volume() const     { return volumeDb_; }
    AlertLevel  level() const      { return level_; }
    const std::string& zone() const { return zoneName_; }
    std::string pattern() const {
        if (level_ == ALERT_EMERGENCY) return "continuous";
        if (level_ == ALERT_WARNING)   return "pulsed";
        return "silent";
    }
};

// -------------------------------------- [U4.3] custom stream manipulators
// Manipulators make the table code short and keep the layout in one place.

// (1) Parameterised manipulator: col(12) / colRight(8) set alignment and width
//     of the NEXT item. Implemented with a small helper struct.
struct ColumnSpec {
    int  width;
    bool leftAligned;
};
inline ColumnSpec col(int width)      { ColumnSpec c = { width, true };  return c; }
inline ColumnSpec colRight(int width) { ColumnSpec c = { width, false }; return c; }
inline std::ostream& operator<<(std::ostream& os, const ColumnSpec& c) {
    if (c.leftAligned) os << std::left;
    else               os << std::right;
    return os << std::setw(c.width);
}

// (2) Plain manipulators: ordinary functions taking and returning an ostream&.
inline std::ostream& alertRule(std::ostream& os) { return os << std::string(78, '-') << '\n'; }

inline std::ostream& nodeHeader(std::ostream& os) {
    return os << col(5) << "ID" << col(18) << "Node" << col(13) << "Type"
              << colRight(7) << "Hazard" << "  " << col(11) << "Level" << "Status" << '\n';
}
inline std::ostream& zoneHeader(std::ostream& os) {
    return os << col(20) << "Zone" << colRight(9) << "Stations" << colRight(7) << "Peak"
              << colRight(7) << "Mean" << "  " << col(11) << "Level" << "Siren" << '\n';
}
inline std::ostream& dispatchHeader(std::ostream& os) {
    return os << col(6) << "Rank" << col(20) << "Zone" << col(11) << "Level"
              << colRight(7) << "Peak" << colRight(9) << "People" << "  " << "Shelter" << '\n';
}
inline std::ostream& logHeader(std::ostream& os) {
    return os << col(7) << "Cycle" << col(7) << "Node" << col(11) << "Level"
              << colRight(7) << "Hazard" << "  " << "Note" << '\n';
}

// -------------------------------------- [U4.3] evacuation priority functors
struct EvacuationTask {
    std::string zone;
    AlertLevel  level;
    double      hazard;
    int         population;
    std::string shelter;
};

// Functor #1 (comparison object for std::sort): highest priority first.
// Priority = higher alert level, then higher hazard, then more people at risk.
struct EvacuationPriority {
    bool operator()(const EvacuationTask& a, const EvacuationTask& b) const {
        if (a.level != b.level)   return a.level > b.level;
        if (a.hazard != b.hazard) return a.hazard > b.hazard;
        return a.population > b.population;
    }
};

// Functor #2 (for std::for_each): it has STATE (rank and people counter), which
// a plain function could not keep between calls.
class DispatchPrinter {
private:
    std::ostream& os_;
    int rank_;
    int totalPeople_;
public:
    explicit DispatchPrinter(std::ostream& os) : os_(os), rank_(0), totalPeople_(0) {}
    void operator()(const EvacuationTask& t) {
        ++rank_;
        totalPeople_ += t.population;
        os_ << col(6) << rank_ << col(20) << t.zone << col(11) << levelName(t.level)
            << colRight(7) << fmt(t.hazard) << colRight(9) << t.population
            << "  " << t.shelter << '\n';
    }
    int totalPeople() const { return totalPeople_; }
};

// --------------------------------------------- [U4.2] binary incident log
// A plain fixed-size record (no std::string, no pointers) so it can be written
// to disk as raw bytes.
struct IncidentRecord {
    int          cycle;
    int          nodeId;
    int          level;
    double       hazard;
    char         note[40];
    unsigned int checksum;     // protects the record against corruption
};

inline unsigned int mixBytes(unsigned int sum, const void* data, std::size_t length) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < length; ++i) sum = sum * 31u + p[i];
    return sum;
}

inline unsigned int recordChecksum(const IncidentRecord& r) {
    unsigned int sum = 17u;
    sum = mixBytes(sum, &r.cycle,  sizeof(r.cycle));
    sum = mixBytes(sum, &r.nodeId, sizeof(r.nodeId));
    sum = mixBytes(sum, &r.level,  sizeof(r.level));
    sum = mixBytes(sum, &r.hazard, sizeof(r.hazard));
    sum = mixBytes(sum, r.note,    sizeof(r.note));
    return sum;
}

inline IncidentRecord makeRecord(int cycle, int nodeId, AlertLevel level, double hazard,
                                 const std::string& note) {
    IncidentRecord r;
    std::memset(&r, 0, sizeof(r));
    r.cycle  = cycle;
    r.nodeId = nodeId;
    r.level  = static_cast<int>(level);
    r.hazard = hazard;
    std::strncpy(r.note, note.c_str(), sizeof(r.note) - 1);
    r.checksum = recordChecksum(r);
    return r;
}

// File layout:   [ int committedCount ][ record 0 ][ record 1 ] ...
// The count in the header decides which records are "official". Records written
// after the last commit are ignored, which is what makes ROLLBACK possible.
class IncidentLogger {
private:
    std::fstream file_;
    std::string  path_;
    int          committed_;       // records that are officially in the log
    int          pending_;         // records written in the current transaction
    bool         inTransaction_;

    std::streamoff offsetOf(int index) const {
        return static_cast<std::streamoff>(sizeof(int)) +
               static_cast<std::streamoff>(index) * static_cast<std::streamoff>(sizeof(IncidentRecord));
    }
    void writeHeader() {
        file_.clear();
        file_.seekp(0, std::ios::beg);                                   // go to the start
        file_.write(reinterpret_cast<const char*>(&committed_), sizeof(committed_));
        file_.flush();
    }
public:
    explicit IncidentLogger(const std::string& path)
        : path_(path), committed_(0), pending_(0), inTransaction_(false) {
        // in | out | binary : read AND write raw bytes; trunc : start with an empty file
        file_.open(path.c_str(), std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
        if (!file_) throw std::runtime_error("cannot open incident log: " + path);
        writeHeader();
    }
    ~IncidentLogger() { if (file_.is_open()) file_.close(); }

    int committedCount() const { return committed_; }
    const std::string& path() const { return path_; }

    void begin() {
        inTransaction_ = true;
        pending_ = 0;
        file_.clear();
        file_.seekp(offsetOf(committed_), std::ios::beg);   // new records go right after the last official one
    }

    void append(const IncidentRecord& r) {
        if (!inTransaction_) throw std::logic_error("append() called outside a transaction");
        if (r.hazard > 100.0) throw ThresholdOverflowException("incident record", r.hazard, 100.0);
        file_.clear();
        file_.seekp(offsetOf(committed_ + pending_), std::ios::beg);
        file_.write(reinterpret_cast<const char*>(&r), sizeof(r));
        ++pending_;
    }

    // COMMIT: updating the header makes the pending records official.
    void commit() {
        committed_ += pending_;
        pending_ = 0;
        inTransaction_ = false;
        writeHeader();
    }

    // ROLLBACK: forget the pending records and move the write pointer back to
    // the end of the officially committed data. The stale bytes are simply
    // overwritten by the next transaction.
    void rollback() {
        pending_ = 0;
        inTransaction_ = false;
        file_.clear();
        file_.seekp(offsetOf(committed_), std::ios::beg);
    }

    // Reads every committed record back and verifies its checksum.
    std::vector<IncidentRecord> readAll() {
        std::vector<IncidentRecord> records;
        file_.clear();
        int count = 0;
        file_.seekg(0, std::ios::beg);
        file_.read(reinterpret_cast<char*>(&count), sizeof(count));
        for (int i = 0; i < count; ++i) {
            IncidentRecord r;
            file_.seekg(offsetOf(i), std::ios::beg);
            file_.read(reinterpret_cast<char*>(&r), sizeof(r));
            if (!file_) throw std::runtime_error("incident log is truncated");
            unsigned int expected = recordChecksum(r);
            if (expected != r.checksum)
                throw TelemetryChecksumMismatch("incident log record #" + toStr(i), r.checksum, expected);
            records.push_back(r);
        }
        return records;
    }

    // TEST HELPER: flips the bits of one byte inside a record (simulated disk
    // corruption). Calling it a second time on the same record repairs it.
    void flipByteForTest(int recordIndex) {
        std::streamoff pos = offsetOf(recordIndex) + static_cast<std::streamoff>(offsetof(IncidentRecord, note));
        char b = 0;
        file_.clear();
        file_.seekg(pos, std::ios::beg);
        file_.read(&b, 1);
        b = static_cast<char>(b ^ 0x5A);
        file_.seekp(pos, std::ios::beg);
        file_.write(&b, 1);
        file_.flush();
    }
};

} // namespace Alerts
} // namespace DisasterCore

using namespace DisasterCore;
using namespace DisasterCore::Sensors;
using namespace DisasterCore::Alerts;

/* ============================================================================
 *  UNIT I DEMONSTRATION
 * ========================================================================== */
void demoUnitI() {
    std::cout << "\n=== UNIT I : inline functions, overloading, references ===\n";

    // [U1.3] Same function name, three different parameter lists.
    std::cout << "computeHazardIndex(rain 65, slope 35)           = "
              << fmt(computeHazardIndex(65.0, 35.0)) << "\n";
    std::cout << "computeHazardIndex(soil 80, vib 12, tilt 3)     = "
              << fmt(computeHazardIndex(80.0, 12.0, 3.0)) << "\n";
    std::cout << "computeHazardIndex(river 5.5 m)                 = "
              << fmt(computeHazardIndex(5.5)) << "\n";

    // [U1.4] Return by reference allows chaining; the SAME object is changed.
    GeoCoordinate site(30.7346, 79.0669);
    std::cout << "coordinate before        : " << fmt(site.latitude, 5) << ", " << fmt(site.longitude, 5) << "\n";
    snapToGrid(shiftCoordinate(site, 0.00123, -0.00087), 0.001);
    std::cout << "after shift + snap (0.001): " << fmt(site.latitude, 5) << ", " << fmt(site.longitude, 5) << "\n";

    // [U1.4] Telemetry state accumulation by reference.
    TelemetryState state;
    TelemetryReading r1 = makeBaseline(40.0, 30.0, 60.0, 10.0, 2.0, 0.0);
    TelemetryReading r2 = makeBaseline(90.0, 30.0, 85.0, 22.0, 4.0, 0.0);
    accumulate(accumulate(state, r1), r2);
    std::cout << "accumulated " << state.samples << " readings, peak hazard "
              << fmt(state.peakHazard) << ", peak vibration " << fmt(state.peakVibration) << " Hz\n";
    peakHazardOf(state) = 0.0;     // a function returning a reference can be assigned to
    std::cout << "peak hazard after peakHazardOf(state) = 0 : " << fmt(state.peakHazard) << "\n";
}

/* ============================================================================
 *  THE GRID CONTROLLER (ties every part together)
 * ========================================================================== */
struct ZoneProfile {
    int         population;
    std::string shelter;
    ZoneProfile(int p = 0, const std::string& s = "") : population(p), shelter(s) {}
};

struct NodeResult {
    int         id;
    std::string name;
    std::string type;
    double      hazard;
    AlertLevel  level;
    std::string status;     // OK, FAULT, BAD-CRC, OVERFLOW
    std::string message;    // exception text, if any
    NodeResult() : id(0), hazard(0.0), level(ALERT_NORMAL), status("OK") {}
};

// Functor used with std::sort to rank stations: uses SensorStation::operator>
struct StationHigherHazard {
    bool operator()(const SensorStation* a, const SensorStation* b) const { return *a > *b; }
};

class WarningGrid {
private:
    std::vector<BaseSensorNode*>             nodes_;      // [U3.4] base-class pointers
    std::vector<SensorStation*>              stations_;   // typed view of the same stations
    AutonomousHybridProbe*                   probe_;
    std::map<int, TelemetryState>            history_;    // [U4.3] map: node id -> running totals
    std::map<std::string, ZoneProfile>       profiles_;
    std::map<std::string, EarlyWarningSiren> sirens_;
    IncidentLogger                           logger_;

public:
    explicit WarningGrid(const std::string& logPath) : probe_(0), logger_(logPath) {}

    WarningGrid(const WarningGrid&) = delete;               // owns raw pointers: no copying
    WarningGrid& operator=(const WarningGrid&) = delete;

    // Deleting through BaseSensorNode* works because the destructor is virtual.
    ~WarningGrid() {
        std::cout << "\n=== Shutting down: deleting every node through its BaseSensorNode* ===\n";
        SimEnvironment::traceLifecycle = true;
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            std::cout << "  deleting node " << nodes_[i]->id() << " (" << nodes_[i]->name() << ")\n";
            delete nodes_[i];
        }
        SimEnvironment::traceLifecycle = false;
    }

    const std::vector<SensorStation*>& stations() const { return stations_; }
    IncidentLogger& logger() { return logger_; }

    void deploy() {
        std::cout << "\n=== UNIT II / III : deploying the sensor network ===\n";
        //                                   rain  slope soil  vib  tilt river
        stations_.push_back(new SensorStation(101, "Kedar_Ridge", "Alaknanda_Valley",
            GeoCoordinate(30.7352, 79.0669), 90.0, makeBaseline(100.0, 42.0, 45.0,  8.0, 2.0, 0.0)));
        stations_.push_back(new SensorStation(102, "Chamoli_Slope", "Alaknanda_Valley",
            GeoCoordinate(30.4000, 79.3200), 80.0, makeBaseline( 85.0, 35.0, 50.0, 10.0, 2.5, 0.0)));
        stations_.push_back(new SensorStation(103, "Bhagirathi_Gauge", "Bhagirathi_Valley",
            GeoCoordinate(30.7300, 78.4500), 11.0, makeBaseline( 60.0, 20.0, 40.0,  4.0, 1.0, 2.5)));
        stations_.push_back(new SensorStation(104, "Tehri_Seismic", "Bhagirathi_Valley",
            GeoCoordinate(30.3780, 78.4800), 85.0, makeBaseline( 70.0, 30.0, 55.0, 12.0, 2.0, 2.2)));

        for (std::size_t i = 0; i < stations_.size(); ++i)
            nodes_.push_back(stations_[i]);     // SensorStation* converts to BaseSensorNode*

        // Build the probe with tracing on to SEE the diamond's construction order.
        std::cout << "Creating AutonomousHybridProbe (watch the constructor order):\n";
        SimEnvironment::traceLifecycle = true;
        probe_ = new AutonomousHybridProbe(201, "Hybrid_Probe_X1", GeoCoordinate(30.5500, 79.0000),
                                           60.0, 20.0, 3);
        SimEnvironment::traceLifecycle = false;
        nodes_.push_back(probe_);

        profiles_.insert(std::make_pair(std::string("Alaknanda_Valley"), ZoneProfile(14000, "Shelter-A")));
        profiles_.insert(std::make_pair(std::string("Bhagirathi_Valley"), ZoneProfile(9500, "Shelter-B")));
        sirens_.insert(std::make_pair(std::string("Alaknanda_Valley"), EarlyWarningSiren("Alaknanda_Valley")));
        sirens_.insert(std::make_pair(std::string("Bhagirathi_Valley"), EarlyWarningSiren("Bhagirathi_Valley")));

        std::cout << "Network ready: " << nodes_.size() << " nodes, "
                  << SensorStation::activeStations() << " active stations (static counter)\n";
    }

    // [U3.2] Proves that the diamond contains only ONE shared root object.
    void showDiamondProof() const {
        std::cout << "\n=== UNIT III : diamond problem check ===\n";
        SolarPoweredNode*        viaSolar = probe_;
        WirelessLoRaTransceiver* viaLoRa  = probe_;
        DisasterTelemetryDevice* rootA = viaSolar;     // implicit up-casts to the virtual base
        DisasterTelemetryDevice* rootB = viaLoRa;
        std::cout << "Root reached through SolarPoweredNode      : " << static_cast<const void*>(rootA) << "\n";
        std::cout << "Root reached through WirelessLoRaTransceiver: " << static_cast<const void*>(rootB) << "\n";
        std::cout << (rootA == rootB ? "-> SAME object: only one DisasterTelemetryDevice exists (virtual base works)\n"
                                     : "-> different objects (diamond problem!)\n");
        std::cout << "Probe serial (no ambiguity): " << probe_->serial()
                  << ",  battery of the CONTAINED PowerSubsystem: " << fmt(probe_->batteryLevel()) << " %\n";
    }

    // One complete sweep of the whole network.
    void runSweep(int cycle, double stormIntensity, bool interference) {
        SimEnvironment::stormIntensity   = stormIntensity;
        SimEnvironment::linkInterference = interference;
        SensorStation::resetGlobalAlert();

        std::cout << "\n" << alertRule << "SWEEP CYCLE " << cycle << "   storm intensity "
                  << fmt(stormIntensity, 1) << (interference ? "   (radio interference!)" : "") << "\n"
                  << alertRule << nodeHeader;

        std::map<int, NodeResult> results;

        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            BaseSensorNode* node = nodes_[i];

            NodeResult res;
            res.id   = node->id();
            res.name = node->name();
            res.type = node->nodeType();           // virtual call

            try {
                // [U3.4] Both calls go through a BASE pointer. Which version runs
                // (SensorStation's or AutonomousHybridProbe's) is decided at RUN TIME
                // through the object's hidden vptr -> vtable.
                TelemetryReading reading = node->readTelemetry();
                res.hazard = overallHazard(reading);
                accumulate(history_[res.id], reading);
                res.level = node->evaluateThreshold();
            }
            catch (const SensorFailureException& e) {          // [U4.1]
                res.status = "FAULT";
                res.message = e.what();
            }
            catch (const TelemetryChecksumMismatch& e) {
                res.status = "BAD-CRC";
                res.message = e.what();
            }
            catch (const ThresholdOverflowException& e) {
                res.status = "OVERFLOW";
                res.level = ALERT_EMERGENCY;       // fail-safe: out-of-range counts as an emergency
                res.message = e.what();
            }
            catch (const std::exception& e) {
                res.status = "ERROR";
                res.message = e.what();
            }

            results[res.id] = res;

            bool noData = (res.status == "FAULT" || res.status == "BAD-CRC" || res.status == "ERROR");
            std::cout << col(5) << res.id << col(18) << res.name << col(13) << res.type
                      << colRight(7) << (noData ? std::string("--") : fmt(res.hazard)) << "  "
                      << col(11) << (noData ? "--" : levelName(res.level)) << res.status << "\n";
            if (!res.message.empty()) std::cout << "      ! " << res.message << "\n";
        }

        // ---- build regional zones with the overloaded + operator [U2.4]
        std::map<std::string, RegionalZone> zones;
        std::vector<SensorStation*> usable;
        for (std::size_t i = 0; i < stations_.size(); ++i) {
            SensorStation* s = stations_[i];
            const std::string& status = results[s->id()].status;
            if (status != "OK" && status != "OVERFLOW") continue;      // no fresh data from this station
            usable.push_back(s);
            std::map<std::string, RegionalZone>::iterator it = zones.find(s->zone());
            if (it == zones.end())
                it = zones.insert(std::make_pair(s->zone(), RegionalZone(s->zone()))).first;
            it->second = it->second + *s;
        }

        // ---- zone summary and sirens
        std::cout << "\n" << zoneHeader;
        std::vector<EvacuationTask> tasks;
        for (std::map<std::string, RegionalZone>::iterator it = zones.begin(); it != zones.end(); ++it) {
            const RegionalZone& z = it->second;
            EarlyWarningSiren& siren = sirens_[z.name()];
            siren.trigger(z.level());
            std::cout << col(20) << z.name() << colRight(9) << z.stationCount()
                      << colRight(7) << fmt(z.peakHazard()) << colRight(7) << fmt(z.meanHazard())
                      << "  " << col(11) << levelName(z.level()) << siren.pattern();
            if (siren.isSounding()) std::cout << " (" << siren.volume() << " dB)";
            std::cout << "\n";

            if (z.level() >= ALERT_WARNING) {
                const ZoneProfile& p = profiles_[z.name()];
                EvacuationTask t;
                t.zone = z.name();  t.level = z.level();  t.hazard = z.peakHazard();
                t.population = p.population;  t.shelter = p.shelter;
                tasks.push_back(t);
            }
        }

        // ---- priority ranking of stations using operator>   [U4.3] std::sort + functor
        if (!usable.empty()) {
            std::sort(usable.begin(), usable.end(), StationHigherHazard());
            std::cout << "Top-priority station: " << usable.front()->name()
                      << " (hazard " << fmt(usable.front()->hazardIndex()) << ")\n";
        }

        // ---- evacuation dispatch: std::sort with a functor, std::for_each with a functor
        if (tasks.empty()) {
            std::cout << "No evacuation required this cycle.\n";
        } else {
            std::sort(tasks.begin(), tasks.end(), EvacuationPriority());
            std::cout << "\nEVACUATION DISPATCH (highest priority first)\n" << dispatchHeader;
            DispatchPrinter done = std::for_each(tasks.begin(), tasks.end(), DispatchPrinter(std::cout));
            std::cout << "People to evacuate: " << done.totalPeople() << "\n";
        }
        std::cout << "Network-wide alert level (static): " << levelName(SensorStation::globalAlert())
                  << "   | active stations (static): " << SensorStation::activeStations() << "\n";

        // ---- persist serious incidents in ONE transaction   [U4.2]
        logger_.begin();
        try {
            int written = 0;
            for (std::map<int, NodeResult>::iterator it = results.begin(); it != results.end(); ++it) {
                const NodeResult& r = it->second;
                if (r.level < ALERT_WARNING) continue;
                std::string note = (r.status == "OVERFLOW") ? "range overflow (fail-safe)" : "threshold exceeded";
                logger_.append(makeRecord(cycle, r.id, r.level, r.hazard, note));
                ++written;
            }
            logger_.commit();
            std::cout << "Incident log: " << written << " record(s) committed (total "
                      << logger_.committedCount() << ")\n";
        }
        catch (const std::exception& e) {
            logger_.rollback();
            std::cout << "Incident log: transaction rolled back - " << e.what() << "\n";
        }
    }

    void printTelemetrySummary() const {
        std::cout << "\n=== Telemetry state accumulated BY REFERENCE during the sweeps ===\n";
        std::cout << col(18) << "Node" << colRight(9) << "Samples" << colRight(12) << "Avg rain"
                  << colRight(11) << "Peak haz." << colRight(11) << "Peak vib." << "\n";
        for (std::map<int, TelemetryState>::const_iterator it = history_.begin(); it != history_.end(); ++it) {
            const TelemetryState& s = it->second;
            std::string name;
            for (std::size_t i = 0; i < nodes_.size(); ++i)
                if (nodes_[i]->id() == it->first) name = nodes_[i]->name();
            std::cout << col(18) << name << colRight(9) << s.samples
                      << colRight(12) << fmt(s.samples ? s.rainfallSum / s.samples : 0.0)
                      << colRight(11) << fmt(s.peakHazard) << colRight(11) << fmt(s.peakVibration) << "\n";
        }
    }
};

/* ============================================================================
 *  UNIT II DEMONSTRATION (buffers, statics, operators)
 * ========================================================================== */
void demoUnitII(const WarningGrid& grid) {
    std::cout << "\n=== UNIT II : deep copy, static members, operators ===\n";

    // [U2.3] deep copy: the copy owns its OWN heap array
    TelemetryBuffer original("demo_buffer", 4);
    original.push(10.0);  original.push(20.0);  original.push(30.0);
    TelemetryBuffer copy(original);                // deep copy constructor
    original.push(100.0);                          // changes only the original
    std::cout << "original: avg " << fmt(original.average()) << "  data at " << original.dataAddress() << "\n";
    std::cout << "copy    : avg " << fmt(copy.average())     << "  data at " << copy.dataAddress() << "\n";
    std::cout << (original.dataAddress() != copy.dataAddress()
                  ? "-> different addresses and different averages: it is a true deep copy\n"
                  : "-> shared memory (shallow copy bug!)\n");

    const std::vector<SensorStation*>& st = grid.stations();

    // [U2.2] the static counter follows the life of every station object
    std::cout << "active stations before clone: " << SensorStation::activeStations() << "\n";
    {
        SensorStation clone(*st[0]);               // station copy constructor
        std::cout << "active stations with a clone : " << SensorStation::activeStations() << "\n";
    }
    std::cout << "active stations after clone is gone: " << SensorStation::activeStations() << "\n";

    // [U2.4] binary + : two stations -> one regional zone
    RegionalZone pair = *st[0] + *st[1];
    std::cout << "\n" << st[0]->name() << " + " << st[1]->name() << " -> zone '" << pair.name() << "', "
              << pair.stationCount() << " stations, peak " << fmt(pair.peakHazard())
              << ", mean " << fmt(pair.meanHazard()) << ", level " << levelName(pair.level()) << "\n";

    // [U2.4] relational > : higher hazard wins
    std::cout << st[0]->name() << " (" << fmt(st[0]->hazardIndex()) << ") > " << st[1]->name()
              << " (" << fmt(st[1]->hazardIndex()) << ") ?  " << (*st[0] > *st[1] ? "yes" : "no") << "\n";

    // [U2.4] stream insertion to console and to a file, extraction from the file
    std::cout << "\nStation state via operator<< to the console:\n  " << *st[0] << "\n";
    {
        std::ofstream out("stations_snapshot.txt");
        for (std::size_t i = 0; i < st.size(); ++i) out << *st[i] << "\n";
    }
    std::cout << "All " << st.size() << " stations written to stations_snapshot.txt\n";

    SensorStation restored;                        // default-constructed target
    std::ifstream in("stations_snapshot.txt");
    in >> restored;                                // operator>> reads the first line back
    std::cout << "Read back with operator>>:\n  " << restored << "\n";
    std::cout << "hazard original " << fmt(st[0]->hazardIndex(), 2) << "  vs  restored "
              << fmt(restored.hazardIndex(), 2) << " (file keeps 3 decimals)\n";

    // [U1.5] the friend class reads the private raw metrics
    std::cout << "\nTelemetryAuditor (friend) inspecting private raw metrics:\n";
    for (std::size_t i = 0; i < st.size(); ++i) TelemetryAuditor::audit(*st[i], std::cout);

    std::cout << "\nAverage of the last 5 hazard values kept in each station's heap buffer:\n";
    for (std::size_t i = 0; i < st.size(); ++i)
        std::cout << "  " << col(18) << st[i]->name() << fmt(st[i]->recentAverageHazard()) << "\n";
}

/* ============================================================================
 *  UNIT IV DEMONSTRATION (binary file, rollback, checksum)
 * ========================================================================== */
void printLog(const std::vector<IncidentRecord>& records) {
    std::cout << logHeader;
    for (std::size_t i = 0; i < records.size(); ++i) {
        const IncidentRecord& r = records[i];
        std::cout << col(7) << r.cycle << col(7) << r.nodeId
                  << col(11) << levelName(static_cast<AlertLevel>(r.level))
                  << colRight(7) << fmt(r.hazard) << "  " << r.note << "\n";
    }
}

void demoUnitIV(WarningGrid& grid) {
    IncidentLogger& log = grid.logger();
    std::cout << "\n=== UNIT IV : binary incident log (" << log.path() << ") ===\n";

    // 1. read everything back from disk
    std::vector<IncidentRecord> records = log.readAll();
    std::cout << "Read " << records.size() << " committed record(s) from disk:\n";
    printLog(records);

    // 2. transactional rollback
    std::cout << "\n-- Transaction test: two valid records, then one invalid (hazard 140) --\n";
    int before = log.committedCount();
    log.begin();
    try {
        log.append(makeRecord(99, 901, ALERT_WARNING, 60.0, "test record 1"));
        log.append(makeRecord(99, 902, ALERT_EMERGENCY, 90.0, "test record 2"));
        log.append(makeRecord(99, 903, ALERT_EMERGENCY, 140.0, "invalid record"));   // throws
        log.commit();
    }
    catch (const ThresholdOverflowException& e) {
        std::cout << "Caught: " << e.what() << "\n";
        log.rollback();                      // seekp() back to the end of the committed data
        std::cout << "Rolled back. Committed records: before=" << before
                  << " after=" << log.committedCount() << " (unchanged)\n";
    }
    std::cout << "Re-reading the file: " << log.readAll().size() << " record(s), none of the test records\n";

    // 3. checksum protection
    if (log.committedCount() > 0) {
        std::cout << "\n-- Corruption test: flip one byte of record #0 on disk --\n";
        log.flipByteForTest(0);
        try {
            log.readAll();
            std::cout << "corruption NOT detected (should not happen)\n";
        }
        catch (const TelemetryChecksumMismatch& e) {
            std::cout << "Caught: " << e.what() << "\n";
        }
        log.flipByteForTest(0);              // flip the same byte again = repair
        std::cout << "After repair the log reads cleanly again: " << log.readAll().size() << " record(s)\n";
    }
}

/* ============================================================================
 *  MAIN
 * ========================================================================== */
int main() {
    std::cout << "============================================================\n"
              << "  ALF-EWG : Autonomous Landslide & Flash-Flood Early Warning Grid\n"
              << "  CIC-211 Object-Oriented Programming Using C++\n"
              << "============================================================\n";

    demoUnitI();

    {   // The grid lives only inside this block, so its destructor runs at the closing brace.
        WarningGrid grid("alf_ewg_incidents.bin");
        grid.deploy();
        grid.showDiamondProof();

        // A storm builds up over five sweep cycles. Radio interference hits in cycle 4.
        const double storm[5] = { 0.3, 0.6, 0.9, 1.3, 1.8 };
        for (int cycle = 1; cycle <= 5; ++cycle)
            grid.runSweep(cycle, storm[cycle - 1], cycle == 4);

        grid.printTelemetrySummary();
        demoUnitII(grid);
        demoUnitIV(grid);
    }

    std::cout << "\nActive stations after shutdown: " << SensorStation::activeStations()
              << "  (0 = every station object was destroyed)\n";
    return 0;
}
