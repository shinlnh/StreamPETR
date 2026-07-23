.pragma library

// function showWarningDialog(dialog, settingsState) {
//     if (settingsState.lksEnabled) {
//         dialog.dialogMessage = "Turn off LKS"
//     } else if (settingsState.accEnabled) {
//         dialog.dialogMessage = "Turn off ACC"
//     } else if (settingsState.reverseMode) {
//         dialog.dialogMessage = "Shift to Drive"
//     }
    
//     dialog.open()
// }

function showWarningDialog(dialog, message) {
    dialog.dialogMessage = message
    dialog.open()
}