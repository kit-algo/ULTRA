#pragma once

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <set>

#include "Entities/Agency.h"
#include "Entities/Calendar.h"
#include "Entities/CalendarDate.h"
#include "Entities/Frequency.h"
#include "Entities/Route.h"
#include "Entities/Stop.h"
#include "Entities/StopTime.h"
#include "Entities/Transfer.h"
#include "Entities/Trip.h"

#include "../Container/Map.h"

#include "../Geometry/Rectangle.h"

#include "../../Helpers/Types.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/IO/ParserCSV.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/FileSystem/FileSystem.h"

namespace GTFS {

class Data {

private:
    Data() {}

public:
    inline static Data FromBinary(const std::string& fileName) noexcept {
        Data data;
        data.deserialize(fileName);
        return data;
    }

    inline static Data FromGTFS(const std::string& fileNameBase, const bool verbose = true) noexcept {
        Data data;
        data.readAgencies(fileNameBase + "agency.txt", verbose);
        data.readCalendars(fileNameBase + "calendar.txt", verbose);
        data.readCalendarDates(fileNameBase + "calendar_dates.txt", verbose);
        data.readFrequencies(fileNameBase + "frequencies.txt", verbose);
        data.readRoutes(fileNameBase + "routes.txt", verbose);
        data.readStops(fileNameBase + "stops.txt", verbose);
        data.readStopTimes(fileNameBase + "stop_times.txt", verbose);
        data.readTransfers(fileNameBase + "transfers.txt", verbose);
        data.readTrips(fileNameBase + "trips.txt", verbose);
        return data;
    }

protected:
    inline void readAgencies(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Agencies", [&](){
            int count = 0;
            IO::CSVReader<3, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "agency_id", "agency_name", "agency_timezone");
            Agency agency;
            while (in.readRow(agency.agencyId, agency.name, agency.timezone)) {
                if (agency.validate()) agencies.emplace_back(agency);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readCalendars(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Calendars", [&](){
            int count = 0;
            IO::CSVReader<10, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "service_id", "sunday", "monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "start_date", "end_date");
            Calendar calendar;
            std::string startDate;
            std::string endDate;
            while (in.readRow(calendar.serviceId, calendar.operatesOnWeekday[0], calendar.operatesOnWeekday[1], calendar.operatesOnWeekday[2], calendar.operatesOnWeekday[3], calendar.operatesOnWeekday[4], calendar.operatesOnWeekday[5], calendar.operatesOnWeekday[6], startDate, endDate)) {
                calendar.startDate = stringToDay(startDate);
                calendar.endDate = stringToDay(endDate);
                if (calendar.validate()) calendars.emplace_back(calendar);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readCalendarDates(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Calendar Dates", [&](){
            int count = 0;
            IO::CSVReader<3, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "service_id", "date", "exception_type");
            CalendarDate calendarDate;
            std::string date;
            int exceptionType = 1;
            while (in.readRow(calendarDate.serviceId, date, exceptionType)) {
                calendarDate.date = stringToDay(date);
                calendarDate.operates = (exceptionType == 1);
                if (calendarDate.validate()) calendarDates.emplace_back(calendarDate);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readFrequencies(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Frequencies", [&](){
            int count = 0;
            IO::CSVReader<4, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "trip_id", "start_time", "end_time", "headway_secs");
            Frequency frequency;
            std::string startTime;
            std::string endTime;
            while (in.readRow(frequency.tripId, startTime, endTime, frequency.headwaySecs)) {
                frequency.startTime = String::parseSeconds(startTime);
                frequency.endTime = String::parseSeconds(endTime);
                if (frequency.validate()) frequencies.emplace_back(frequency);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readRoutes(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Routes", [&](){
            int count = 0;
            IO::CSVReader<7, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "route_id", "agency_id", "route_short_name", "route_long_name", "route_type", "route_color", "route_text_color");
            Route route;
            std::string shortName;
            std::string longName;
            while (in.readRow(route.routeId, route.agencyId, shortName, longName, route.type, route.routeColor, route.textColor)) {
                route.name = "[" + shortName + "] " + longName;
                if (route.validate()) routes.emplace_back(route);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readStops(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Stops", [&](){
            int count = 0;
            IO::CSVReader<4, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "stop_id", "stop_name", "stop_lat", "stop_lon");
            Stop stop;
            double latitude = 0.0;
            double longitude = 0.0;
            while (in.readRow(stop.stopId, stop.name, latitude, longitude)) {
                stop.coordinates = Geometry::Point(Construct::LatLong, latitude, longitude);
                if (stop.validate()) stops.emplace_back(stop);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readStopTimes(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Stop Times", [&](){
            int count = 0;
            IO::CSVReader<5, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "trip_id", "arrival_time", "departure_time", "stop_id", "stop_sequence");
            StopTime stopTime;
            std::string arrivalTime;
            std::string departureTime;
            while (in.readRow(stopTime.tripId, arrivalTime, departureTime, stopTime.stopId, stopTime.stopSequence)) {
                stopTime.arrivalTime = String::parseSeconds(arrivalTime);
                stopTime.departureTime = String::parseSeconds(departureTime);
                if (stopTime.validate()) stopTimes.push_back(stopTime);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readTransfers(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Transfers", [&](){
            int count = 0;
            IO::CSVReader<4, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "from_stop_id", "to_stop_id", "min_transfer_time", "transfer_type");
            Transfer transfer;
            int transferType = 0;
            while (in.readRow(transfer.fromStopId, transfer.toStopId, transfer.minTransferTime, transferType)) {
                if (transferType == 3) continue;
                if (transfer.validate()) transfers.emplace_back(transfer);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readTrips(const std::string& fileName, const bool verbose = true) {
        IO::readFile(fileName, "Trips", [&](){
            int count = 0;
            IO::CSVReader<4, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileName);
            in.readHeader(ReadMode, "route_id", "service_id", "trip_id", "trip_short_name");
            Trip trip;
            while (in.readRow(trip.routeId, trip.serviceId, trip.tripId, trip.name)) {
                if (trip.validate()) trips.emplace_back(trip);
                count++;
            }
            return count;
        }, verbose);
    }

public:
    inline Map<std::string, std::vector<int>> unrollCalendarDates(const int startDate, const int endDate, const bool ignoreDaysOfOperation = false) const noexcept {
        Map<std::string, std::set<int>> data;
        for (const Calendar& calendar : calendars) {
            std::set<int>& dates = data[calendar.serviceId];
            if (ignoreDaysOfOperation) {
                for (int date = startDate; date <= endDate; date++) {
                    dates.insert(date);
                }
            } else {
                int firstData = std::max(startDate, calendar.startDate);
                int lastData = std::min(endDate, calendar.endDate);
                for (int date = firstData; date <= lastData; date++) {
                    if (calendar.operatesOnWeekday[weekday(date)]) dates.insert(date);
                }
            }
        }
        for (const CalendarDate& calendarDate : calendarDates) {
            std::set<int>& dates = data[calendarDate.serviceId];
            if (ignoreDaysOfOperation) {
                for (int date = startDate; date <= endDate; date++) {
                    dates.insert(date);
                }
            } else {
                if (calendarDate.date < startDate) continue;
                if (calendarDate.date > endDate) continue;
                if (calendarDate.operates) {
                    data[calendarDate.serviceId].insert(calendarDate.date);
                } else {
                    data[calendarDate.serviceId].erase(calendarDate.date);
                }
            }
        }
        int minDate = never;
        for (const auto& dates : data) {
            for (const int date : dates.second) {
                if (minDate > date) minDate = date;
            }
        }
        Map<std::string, std::vector<int>> result;
        for (const auto& dates : data) {
            if (dates.second.empty()) continue;
            std::vector<int>& vector = result[dates.first];
            for (const int date : dates.second) {
                vector.emplace_back(date - minDate);
            }
            std::sort(vector.begin(), vector.end());
        }
        return result;
    }

    inline Map<std::string, int> routeIds() const noexcept {
        Map<std::string, int> ids;
        for (size_t i = 0; i < routes.size(); i++) {
            const Route& route = routes[i];
            if (ids.contains(route.routeId)) continue;
            ids.insert(route.routeId, i);
        }
        return ids;
    }

    inline Map<std::string, int> stopIds() const noexcept {
        Map<std::string, int> ids;
        for (size_t i = 0; i < stops.size(); i++) {
            const Stop& stop = stops[i];
            if (ids.contains(stop.stopId)) continue;
            ids.insert(stop.stopId, i);
        }
        return ids;
    }

    inline Map<std::string, int> tripIds() const noexcept {
        Map<std::string, int> ids;
        for (size_t i = 0; i < trips.size(); i++) {
            const Trip& trip = trips[i];
            if (ids.contains(trip.tripId)) continue;
            ids.insert(trip.tripId, i);
        }
        return ids;
    }

    inline Map<std::string, std::vector<int>> frequencyIds() const noexcept {
        Map<std::string, std::vector<int>> ids;
        for (size_t i = 0; i < frequencies.size(); i++) {
            ids[frequencies[i].tripId].emplace_back(i);
        }
        return ids;
    }

    inline Geometry::Rectangle boundingBox() const noexcept {
        Geometry::Rectangle result = Geometry::Rectangle::Empty();
        for (const Stop& stop : stops) {
            result.extend(stop.coordinates);
        }
        return result;
    }

    inline void printInfo() {
        int firstDay = std::numeric_limits<int>::max();
        int lastDay = std::numeric_limits<int>::min();
        for (const Calendar& calendar : calendars) {
            if (firstDay > calendar.startDate) firstDay = calendar.startDate;
            if (lastDay < calendar.endDate) lastDay = calendar.endDate;
        }
        for (const CalendarDate& calendarDate : calendarDates) {
            if (firstDay > calendarDate.date) firstDay = calendarDate.date;
            if (lastDay < calendarDate.date) lastDay = calendarDate.date;
        }
        std::cout << "GTFS raw data:" << std::endl;
        std::cout << "   Number of Agencies:       " << std::setw(12) << String::prettyInt(agencies.size()) << std::endl;
        std::cout << "   Number of Calendars:      " << std::setw(12) << String::prettyInt(calendars.size()) << std::endl;
        std::cout << "   Number of Calendar Dates: " << std::setw(12) << String::prettyInt(calendarDates.size()) << std::endl;
        std::cout << "   Number of Frequencies:    " << std::setw(12) << String::prettyInt(frequencies.size()) << std::endl;
        std::cout << "   Number of Routes:         " << std::setw(12) << String::prettyInt(routes.size()) << std::endl;
        std::cout << "   Number of Stops:          " << std::setw(12) << String::prettyInt(stops.size()) << std::endl;
        std::cout << "   Number of Stop Times:     " << std::setw(12) << String::prettyInt(stopTimes.size()) << std::endl;
        std::cout << "   Number of Transfers:      " << std::setw(12) << String::prettyInt(transfers.size()) << std::endl;
        std::cout << "   Number of Trips:          " << std::setw(12) << String::prettyInt(trips.size()) << std::endl;
        std::cout << "   First Day:                " << std::setw(12) << dayToString(firstDay) << std::endl;
        std::cout << "   Last Day:                 " << std::setw(12) << dayToString(lastDay) << std::endl;
        std::cout << "   Bounding Box:             " << std::setw(12) << boundingBox() << std::endl;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, agencies, calendars, calendarDates, frequencies, routes, stops, stopTimes, transfers, trips);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, agencies, calendars, calendarDates, frequencies, routes, stops, stopTimes, transfers, trips);
    }

public:
    std::vector<Agency> agencies;
    std::vector<Calendar> calendars;
    std::vector<CalendarDate> calendarDates;
    std::vector<Frequency> frequencies;
    std::vector<Route> routes;
    std::vector<Stop> stops;
    std::vector<StopTime> stopTimes;
    std::vector<Transfer> transfers;
    std::vector<Trip> trips;

protected:
    static constexpr IO::IgnoreColumn ReadMode = IO::IGNORE_EXTRA_COLUMN | IO::IGNORE_MISSING_COLUMN;

};

}
