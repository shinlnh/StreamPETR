pragma Singleton
import QtQuick 2.14

import com.banvien.MainViewController 1.0

QtObject {
    // Driver assistance status
    property bool lksEnabled: false
    property bool accEnabled: false
    property bool aebEnabled: true

    // Gear status
    property int gear: 2  // Default gear to Neutral (2)
}