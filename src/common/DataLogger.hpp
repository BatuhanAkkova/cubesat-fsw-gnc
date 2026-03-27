#pragma once

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace common {

/**
 * @brief Simple CSV data logger for simulation data export
 *
 * Usage:
 *   DataLogger logger("output.csv");
 *   logger.addColumn("time");
 *   logger.addColumn("qw");
 *   logger.addColumn("qx");
 *   logger.writeHeader();
 *
 *   logger.startRow();
 *   logger.addValue(t);
 *   logger.addValue(q.w());
 *   logger.addValue(q.x());
 *   logger.endRow();
 */
class DataLogger {
   public:
    /**
     * @brief Constructor
     * @param filename Output CSV filename
     */
    explicit DataLogger(const std::string& filename) : filename_(filename), file_(filename), row_started_(false) {
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        file_ << std::fixed << std::setprecision(6);
    }

    /**
     * @brief Destructor - ensures file is properly closed
     */
    ~DataLogger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    /**
     * @brief Add a column name (must be called before writeHeader)
     * @param name Column name
     */
    void addColumn(const std::string& name) {
        columns_.push_back(name);
    }

    /**
     * @brief Write the CSV header row
     */
    void writeHeader() {
        for (size_t i = 0; i < columns_.size(); ++i) {
            file_ << columns_[i];
            if (i < columns_.size() - 1) {
                file_ << ",";
            }
        }
        file_ << "\n";
        file_.flush();
    }

    /**
     * @brief Start a new data row
     */
    void startRow() {
        row_started_ = true;
        current_row_.clear();
    }

    /**
     * @brief Add a value to the current row
     * @param value Value to add (any numeric type)
     */
    template <typename T>
    void addValue(T value) {
        if (!row_started_) {
            throw std::runtime_error("Must call startRow() before addValue()");
        }
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << value;
        current_row_.push_back(oss.str());
    }

    /**
     * @brief End the current row and write to file
     */
    void endRow() {
        if (!row_started_) {
            throw std::runtime_error("No row to end");
        }

        if (current_row_.size() != columns_.size()) {
            throw std::runtime_error("Row size mismatch: expected " + std::to_string(columns_.size()) +
                                     " values, got " + std::to_string(current_row_.size()));
        }

        for (size_t i = 0; i < current_row_.size(); ++i) {
            file_ << current_row_[i];
            if (i < current_row_.size() - 1) {
                file_ << ",";
            }
        }
        file_ << "\n";

        row_started_ = false;

        // Flush every 100 rows for reasonable performance
        if (++row_count_ % 100 == 0) {
            file_.flush();
        }
    }

    /**
     * @brief Flush the file buffer
     */
    void flush() {
        file_.flush();
    }

    /**
     * @brief Get the filename
     */
    const std::string& getFilename() const {
        return filename_;
    }

   private:
    std::string filename_;
    std::ofstream file_;
    std::vector<std::string> columns_;
    std::vector<std::string> current_row_;
    bool row_started_;
    size_t row_count_ = 0;
};

}  // namespace common
