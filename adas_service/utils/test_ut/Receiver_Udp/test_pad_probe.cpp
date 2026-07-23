#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"
#include "mocking.h"
#include <gst/gst.h>
#include <gst/rtp/gstrtpbuffer.h>

using ::testing::_;
using ::testing::Return;

/**
 * @file        test_pad_probe.cpp
 * @brief       Test implementation for constructor of ReceiverUdp
 * @details     Includes Coverage tests
 */

/**
 * =============================================================================
 * Test Case Group: Coverage Testing
 * =============================================================================
 * @test        TC_pad_probe_callback_coverage_01
 * 
 * -----------------------------------------------------------------------------
 * Test Category:   Code Coverage
 * Test Type:       Unit Test
 * Coverage Type:   Both
 * -----------------------------------------------------------------------------
 * 
 * Prerequisites:
 *      - A valid `GstBuffer` instance is created.
 *      - Extension data (4 bytes) is added to the buffer.
 * 
 * Test Steps:
 *      1. Create a `GstBuffer` and populate it with valid extension data.
 *      2. Map and fill the buffer using `gst_buffer_map`.
 *      3. Pass the buffer through `pad_probe_callback` using `GstPadProbeInfo`.
 *      4. Verify the function processes the buffer correctly and returns the expected output.
 * 
 * Input:
 *      - A `GstBuffer` instance with 4 bytes of extension data.
 * 
 * Expected Output:
 *      - The function `pad_probe_callback` returns `GST_PAD_PROBE_OK`.
 *      - No exceptions or errors occur during execution.
 * 
 * Actual Output:
 *      - No error codes returned.
 * 
 * Pass/Fail Criteria:
 *      - The function successfully processes the buffer and returns `GST_PAD_PROBE_OK`.
 *      - No exceptions or errors are encountered.
 */

// TEST_F(ReceiverUdpTest, TC_pad_probe_callback_coverage_01) {
//     GstBuffer *buffer = gst_buffer_new_and_alloc(100);
//     GstMapInfo map;

//     gst_buffer_map(buffer, &map, GST_MAP_WRITE);
//     guint8 extension_data[] = {0x90, 0x00, 0x12, 0x34};
//     memcpy(map.data, extension_data, sizeof(extension_data));
//     gst_buffer_unmap(buffer, &map);

//     GstPadProbeInfo info = {};
//     info.data = buffer;

//     GstPadProbeReturn result = pad_probe_callback(nullptr, &info, nullptr);
//     EXPECT_EQ(result, GST_PAD_PROBE_OK);

//     gst_buffer_unref(buffer);
// }
/**
 * =============================================================================
 * Test Case Group: Coverage Testing
 * =============================================================================
 * @test        TC_pad_probe_callback_coverage_02
 * 
 * -----------------------------------------------------------------------------
 * Test Category:   Code Coverage
 * Test Type:       Unit Test
 * Coverage Type:   Both
 * -----------------------------------------------------------------------------
 * 
 * Prerequisites:
 *      - A valid `GstBuffer` instance is created.
 *      - The buffer does not contain any extension data.
 * 
 * Test Steps:
 *      1. Create a `GstBuffer` and populate it with non-extension data (2 bytes).
 *      2. Map and fill the buffer using `gst_buffer_map`.
 *      3. Pass the buffer through `pad_probe_callback` using `GstPadProbeInfo`.
 *      4. Verify the function handles the buffer correctly and returns the expected output.
 * 
 * Input:
 *      - A `GstBuffer` instance with 2 bytes of non-extension data.
 * 
 * Expected Output:
 *      - The function `pad_probe_callback` returns `GST_PAD_PROBE_OK`.
 *      - No exceptions or errors occur during execution.
 * 
 * Actual Output:
 *      - No error codes returned.
 * 
 * Pass/Fail Criteria:
 *      - The function successfully processes the buffer and returns `GST_PAD_PROBE_OK`.
 *      - No exceptions or errors are encountered.
 */

// TEST_F(ReceiverUdpTest, TC_pad_probe_callback_coverage_02) {
//     GstBuffer *buffer = gst_buffer_new_and_alloc(100);
//     GstMapInfo map;

//     gst_buffer_map(buffer, &map, GST_MAP_WRITE);
//     guint8 no_extension_data[] = {0x00, 0x00};
//     memcpy(map.data, no_extension_data, sizeof(no_extension_data));
//     gst_buffer_unmap(buffer, &map);

//     GstPadProbeInfo info = {};
//     info.data = buffer;

//     GstPadProbeReturn result = pad_probe_callback(nullptr, &info, nullptr);
//     EXPECT_EQ(result, GST_PAD_PROBE_OK);

//     gst_buffer_unref(buffer);
// }

/**
 * =============================================================================
 * Test Case Group: Coverage Testing
 * =============================================================================
 * @test        TC_pad_probe_callback_coverage_03
 * 
 * -----------------------------------------------------------------------------
 * Test Category:   Code Coverage
 * Test Type:       Unit Test
 * Coverage Type:   Both
 * -----------------------------------------------------------------------------
 * 
 * Prerequisites:
 *      - A valid `GstBuffer` instance is created.
 *      - The buffer contains 5 bytes of extension data.
 *      - A `ReceiverUdp` object is initialized to interact with the function.
 * 
 * Test Steps:
 *      1. Create a `GstBuffer` and populate it with long extension data (5 bytes).
 *      2. Map and fill the buffer using `gst_buffer_map`.
 *      3. Pass the buffer through `pad_probe_callback` using `GstPadProbeInfo`.
 *      4. Validate that the extension data is extracted and logged correctly.
 *      5. Verify the interaction with the `ReceiverUdp` object.
 * 
 * Input:
 *      - A `GstBuffer` instance with 5 bytes of extension data.
 *      - A `ReceiverUdp` object for interaction.
 * 
 * Expected Output:
 *      - The function `pad_probe_callback` returns `GST_PAD_PROBE_OK`.
 *      - Extension data is extracted and logged successfully.
 *      - The `ReceiverUdp` object's `timeStamp` remains unaffected.
 * 
 * Actual Output:
 *      - No error codes returned.
 * 
 * Pass/Fail Criteria:
 *      - The function successfully processes the buffer and returns `GST_PAD_PROBE_OK`.
 *      - Extension data is handled as expected without errors.
 *      - No exceptions or errors are encountered.
 */

// TEST_F(ReceiverUdpTest, TC_pad_probe_callback_coverage_03) {

//     GstBuffer *buffer = gst_buffer_new_and_alloc(100);
//     GstMapInfo map;

//     gst_buffer_map(buffer, &map, GST_MAP_WRITE);

    
//     guint8 extension_data[] = {0x90, 0x00, 0x12, 0x34, 0x56};
//     memcpy(map.data, extension_data, sizeof(extension_data));

//     gst_buffer_unmap(buffer, &map);

//     GstPadProbeInfo info = {};
//     info.data = buffer;

//     ReceiverUdp receiver(1920, 1080);

//     GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;
//     if (gst_rtp_buffer_map(buffer, GST_MAP_READWRITE, &rtp_buffer)) {
//         guint16 extension_bits;
//         guint8 *extension_data_ptr;
//         guint extension_length_words;

//         if (gst_rtp_buffer_get_extension_data(&rtp_buffer, &extension_bits, (gpointer*)&extension_data_ptr, &extension_length_words)) {
//             g_print("Extension bits: %u\n", extension_bits);
//             g_print("Extension length in words: %u\n", extension_length_words);
//             g_print("Extension data length: %u\n", extension_length_words * 4);
//         } else {
//             g_print("No extension data found in RTP buffer.\n");
//         }

//         gst_rtp_buffer_unmap(&rtp_buffer);
//     } else {
//         g_print("Failed to map RTP buffer.\n");
//     }

//     GstPadProbeReturn result = pad_probe_callback(nullptr, &info, &receiver);

//     EXPECT_EQ(result, GST_PAD_PROBE_OK);
//     EXPECT_EQ(receiver.timeStamp, 0);

//     gst_buffer_unref(buffer);
// }


