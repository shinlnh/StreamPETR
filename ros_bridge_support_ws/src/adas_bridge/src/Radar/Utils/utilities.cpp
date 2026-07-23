#include "utilities.h"
#include <Eigen/Dense>
#include "common.h"

// Global UI state definition
UIState g_ui_state;

// Button structure
struct Button {
    cv::Rect rect;
    std::string label;
    bool is_active;
    
    Button(int x, int y, int w, int h, const std::string& lbl, bool active = false)
        : rect(x, y, w, h), label(lbl), is_active(active) {}
    
    bool contains(int px, int py) const {
        return rect.contains(cv::Point(px, py));
    }
    
    void draw(cv::Mat& img, cv::Scalar color = cv::Scalar(70, 130, 180)) const {
        cv::Scalar bg_color = is_active ? cv::Scalar(100, 180, 100) : color;
        cv::rectangle(img, rect, bg_color, -1);
        cv::rectangle(img, rect, cv::Scalar(200, 200, 200), 2);
        
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
        cv::Point text_pos(
            rect.x + (rect.width - text_size.width) / 2,
            rect.y + (rect.height + text_size.height) / 2
        );
        cv::putText(img, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
};

// Text box structure
struct TextBox {
    cv::Rect rect;
    std::string label;
    std::string* text_ptr;
    bool is_active;
    
    TextBox(int x, int y, int w, int h, const std::string& lbl, std::string* txt)
        : rect(x, y, w, h), label(lbl), text_ptr(txt), is_active(false) {}
    
    bool contains(int px, int py) const {
        return rect.contains(cv::Point(px, py));
    }
    
    void draw(cv::Mat& img) const {
        cv::Scalar bg_color = is_active ? cv::Scalar(60, 60, 80) : cv::Scalar(40, 40, 40);
        cv::Scalar border_color = is_active ? cv::Scalar(100, 200, 255) : cv::Scalar(100, 100, 100);
        
        cv::rectangle(img, rect, bg_color, -1);
        cv::rectangle(img, rect, border_color, 2);
        
        // Draw label above textbox
        cv::putText(img, label, cv::Point(rect.x, rect.y - 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        
        // Draw text content
        std::string display_text = text_ptr ? *text_ptr : "";
        if (is_active) display_text += "_";
        
        cv::putText(img, display_text, cv::Point(rect.x + 5, rect.y + rect.height - 8), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
};

// Global UI elements
static Button* g_toggle_menu_btn = nullptr;
static Button* g_logging_btn = nullptr;
static Button* g_apply_dbscan_btn = nullptr;
static Button* g_apply_filter_btn = nullptr;
static Button* g_log_filter_btn = nullptr;
static TextBox* g_minpoints_textbox = nullptr;
static TextBox* g_epsilon_textbox = nullptr;
static TextBox* g_cutoff_textbox = nullptr;
static TextBox* g_zmin_textbox = nullptr;
static TextBox* g_zmax_textbox = nullptr;
static TextBox* g_log_xmin_textbox = nullptr;
static TextBox* g_log_xmax_textbox = nullptr;
static TextBox* g_log_ymin_textbox = nullptr;
static TextBox* g_log_ymax_textbox = nullptr;
static TextBox* g_log_zmin_textbox = nullptr;
static TextBox* g_log_zmax_textbox = nullptr;
static Button* g_segmented_mode_btn = nullptr;

// Mouse data structure
struct MouseData {
    int mouse_x = -1;
    int mouse_y = -1;
    int origin_x = 0;
    int origin_y = 0;
    float scale = 1.0f;
    bool show_crosshair = false;
};

static MouseData g_mouse_data;

void onMouseEnhanced(int event, int x, int y, int flags, void* userdata)
{
    (void)flags;
    (void)userdata;
    
    if (event == cv::EVENT_MOUSEMOVE)
    {
        g_mouse_data.mouse_x = x;
        g_mouse_data.mouse_y = y;
        g_mouse_data.show_crosshair = true;
    }
    else if (event == cv::EVENT_LBUTTONDOWN)
    {
        // Check toggle menu button
        if (g_toggle_menu_btn && g_toggle_menu_btn->contains(x, y)) {
            g_ui_state.show_menu = !g_ui_state.show_menu;
        }
        
        if (g_ui_state.show_menu) {
            if (g_segmented_mode_btn && g_segmented_mode_btn->contains(x, y)) {
                g_ui_state.use_segmented_clusters_mode = !g_ui_state.use_segmented_clusters_mode;
                RCLCPP_INFO(rclcpp::get_logger("viz_mode"), 
                        "Visualization mode: %s", 
                        g_ui_state.use_segmented_clusters_mode ? "SEGMENTED CLUSTERS" : "FINAL OBJECTS");
            }
            // Check logging button
            if (g_logging_btn && g_logging_btn->contains(x, y)) {
                if (g_ui_state.is_logging) {
                    stopLogging();
                } else {
                    startLogging();
                }
            }
            
            // Check DBSCAN apply button
            if (g_apply_dbscan_btn && g_apply_dbscan_btn->contains(x, y)) {
                try {
                    g_ui_state.temp_min_points = std::stoi(g_ui_state.min_points_text);
                    g_ui_state.temp_epsilon = std::stod(g_ui_state.epsilon_text);
                    
                    // Validate DBSCAN
                    if (g_ui_state.temp_min_points < 1) g_ui_state.temp_min_points = 1;
                    if (g_ui_state.temp_epsilon < 0.01) g_ui_state.temp_epsilon = 0.01;
                    
                    RCLCPP_INFO(rclcpp::get_logger("dbscan_tuning"), 
                               "Applied DBSCAN: MIN=%d, EPS=%.3f",
                               g_ui_state.temp_min_points, g_ui_state.temp_epsilon);
                } catch (...) {
                    RCLCPP_WARN(rclcpp::get_logger("dbscan_tuning"), 
                               "Invalid DBSCAN values!");
                }
            }
            
            // Check Filter apply button (only for preprocessing filters)
            if (g_apply_filter_btn && g_apply_filter_btn->contains(x, y)) {
                try {
                    g_ui_state.temp_cutoff_distance = std::stof(g_ui_state.cutoff_distance_text);
                    g_ui_state.temp_z_min = std::stof(g_ui_state.z_min_text);
                    g_ui_state.temp_z_max = std::stof(g_ui_state.z_max_text);
                    
                    // Validate filter
                    if (g_ui_state.temp_cutoff_distance < 0.1) g_ui_state.temp_cutoff_distance = 0.1;
                    if (g_ui_state.temp_z_min > g_ui_state.temp_z_max) {
                        std::swap(g_ui_state.temp_z_min, g_ui_state.temp_z_max);
                    }
                    
                    RCLCPP_INFO(rclcpp::get_logger("filter_tuning"), 
                               "Applied Filter: Y<%.1f, Z[%.2f,%.2f]",
                               g_ui_state.temp_cutoff_distance, g_ui_state.temp_z_min, g_ui_state.temp_z_max);
                } catch (...) {
                    RCLCPP_WARN(rclcpp::get_logger("filter_tuning"), 
                               "Invalid filter values!");
                }
            }
            
            // Check Log Filter button (Enable/Disable)
            if (g_log_filter_btn && g_log_filter_btn->contains(x, y)) {
                // Prevent changes while logging is active
                if (g_ui_state.is_logging) {
                    RCLCPP_WARN(rclcpp::get_logger("log_filter"), 
                               "Cannot change log filter while logging is active!");
                } else {
                    if (!g_ui_state.use_logging_filter) {
                        // ENABLE: Parse values from text boxes
                        try {
                            g_ui_state.log_x_min = std::stof(g_ui_state.log_x_min_text);
                            g_ui_state.log_x_max = std::stof(g_ui_state.log_x_max_text);
                            g_ui_state.log_y_min = std::stof(g_ui_state.log_y_min_text);
                            g_ui_state.log_y_max = std::stof(g_ui_state.log_y_max_text);
                            g_ui_state.log_z_min = std::stof(g_ui_state.log_z_min_text);
                            g_ui_state.log_z_max = std::stof(g_ui_state.log_z_max_text);
                            
                            g_ui_state.use_logging_filter = true;
                            
                            RCLCPP_INFO(rclcpp::get_logger("log_filter"), 
                                       "Enabled Log Filter: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]",
                                       g_ui_state.log_x_min, g_ui_state.log_x_max,
                                       g_ui_state.log_y_min, g_ui_state.log_y_max,
                                       g_ui_state.log_z_min, g_ui_state.log_z_max);
                        } catch (...) {
                            RCLCPP_ERROR(rclcpp::get_logger("log_filter"), 
                                        "Invalid log filter values! Filter not enabled.");
                        }
                    } else {
                        // DISABLE: Reset to default values (no filtering)
                        g_ui_state.log_x_min = -1000.0f;
                        g_ui_state.log_x_max = 1000.0f;
                        g_ui_state.log_y_min = -1000.0f;
                        g_ui_state.log_y_max = 1000.0f;
                        g_ui_state.log_z_min = -1000.0f;
                        g_ui_state.log_z_max = 1000.0f;
                        
                        g_ui_state.use_logging_filter = false;
                        
                        RCLCPP_INFO(rclcpp::get_logger("log_filter"), 
                                   "Disabled Log Filter (logging all clusters)");
                    }
                }
            }
            
            // Check text boxes
            g_ui_state.active_textbox = UIState::NONE;
            if (g_minpoints_textbox && g_minpoints_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::MIN_POINTS_BOX;
            }
            else if (g_epsilon_textbox && g_epsilon_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::EPSILON_BOX;
            }
            else if (g_cutoff_textbox && g_cutoff_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::CUTOFF_DISTANCE_BOX;
            }
            else if (g_zmin_textbox && g_zmin_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::Z_MIN_BOX;
            }
            else if (g_zmax_textbox && g_zmax_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::Z_MAX_BOX;
            }
            else if (g_log_xmin_textbox && g_log_xmin_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::LOG_X_MIN_BOX;
            }
            else if (g_log_xmax_textbox && g_log_xmax_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::LOG_X_MAX_BOX;
            }
            else if (g_log_ymin_textbox && g_log_ymin_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::LOG_Y_MIN_BOX;
            }
            else if (g_log_ymax_textbox && g_log_ymax_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::LOG_Y_MAX_BOX;
            }
            else if (g_log_zmin_textbox && g_log_zmin_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::LOG_Z_MIN_BOX;
            }
            else if (g_log_zmax_textbox && g_log_zmax_textbox->contains(x, y)) {
                g_ui_state.active_textbox = UIState::LOG_Z_MAX_BOX;
            }
        }
    }
}

void handleKeyboardInput(int key)
{
    if (g_ui_state.active_textbox == UIState::NONE)
        return;
    
    std::string* target_text = nullptr;
    
    switch (g_ui_state.active_textbox) {
        case UIState::MIN_POINTS_BOX:
            target_text = &g_ui_state.min_points_text;
            break;
        case UIState::EPSILON_BOX:
            target_text = &g_ui_state.epsilon_text;
            break;
        case UIState::CUTOFF_DISTANCE_BOX:
            target_text = &g_ui_state.cutoff_distance_text;
            break;
        case UIState::Z_MIN_BOX:
            target_text = &g_ui_state.z_min_text;
            break;
        case UIState::Z_MAX_BOX:
            target_text = &g_ui_state.z_max_text;
            break;
        case UIState::LOG_X_MIN_BOX:
            target_text = &g_ui_state.log_x_min_text;
            break;
        case UIState::LOG_X_MAX_BOX:
            target_text = &g_ui_state.log_x_max_text;
            break;
        case UIState::LOG_Y_MIN_BOX:
            target_text = &g_ui_state.log_y_min_text;
            break;
        case UIState::LOG_Y_MAX_BOX:
            target_text = &g_ui_state.log_y_max_text;
            break;
        case UIState::LOG_Z_MIN_BOX:
            target_text = &g_ui_state.log_z_min_text;
            break;
        case UIState::LOG_Z_MAX_BOX:
            target_text = &g_ui_state.log_z_max_text;
            break;
        default:
            return;
    }
    
    if (!target_text) return;
    
    // Handle backspace
    if (key == 8 && !target_text->empty()) {
        target_text->pop_back();
    }
    // Handle numbers, dot, and minus
    else if ((key >= '0' && key <= '9') || key == '.' || key == '-') {
        if (target_text->length() < 10) {
            *target_text += static_cast<char>(key);
        }
    }
    // Handle Enter
    else if (key == 13) {
        g_ui_state.active_textbox = UIState::NONE;
    }
}

void drawCrosshair(cv::Mat& img, const MouseData& data, int img_width, int img_height)
{
    if (!data.show_crosshair || data.mouse_x < 0 || data.mouse_y < 0)
        return;
    int x = data.mouse_x;
    int y = data.mouse_y;

    cv::line(img, cv::Point(x, 0), cv::Point(x, img_height), 
            cv::Scalar(255, 255, 0), 1, cv::LINE_AA);

    cv::line(img, cv::Point(0, y), cv::Point(img_width, y), 
            cv::Scalar(255, 255, 0), 1, cv::LINE_AA);

    float radar_x = (x - data.origin_x) / data.scale;
    float radar_y = (data.origin_y - y) / data.scale;

    std::ostringstream x_label;
    x_label << std::fixed << std::setprecision(1) << radar_x << "m";
    cv::Size x_text_size = cv::getTextSize(x_label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);

    int x_label_x = x - x_text_size.width / 2;
    int x_label_y = 25;

    if (x_label_x < 5) x_label_x = 5;
    if (x_label_x + x_text_size.width > img_width - 5) 
        x_label_x = img_width - x_text_size.width - 5;

    cv::rectangle(img, 
                cv::Point(x_label_x - 3, x_label_y - x_text_size.height - 3),
                cv::Point(x_label_x + x_text_size.width + 3, x_label_y + 3),
                cv::Scalar(0, 0, 0), -1);

    cv::putText(img, x_label.str(), cv::Point(x_label_x, x_label_y),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);

    std::ostringstream y_label;
    y_label << std::fixed << std::setprecision(1) << radar_y << "m";
    cv::Size y_text_size = cv::getTextSize(y_label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);

    int y_label_x = 10;
    int y_label_y = y + 5;

    if (y_label_y - y_text_size.height < 5) 
        y_label_y = y_text_size.height + 5;
    if (y_label_y > img_height - 5) 
        y_label_y = img_height - 5;

    cv::rectangle(img, 
                cv::Point(y_label_x - 3, y_label_y - y_text_size.height - 3),
                cv::Point(y_label_x + y_text_size.width + 3, y_label_y + 3),
                cv::Scalar(0, 0, 0), -1);

    cv::putText(img, y_label.str(), cv::Point(y_label_x, y_label_y),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);

    cv::circle(img, cv::Point(x, y), 3, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
}

void drawMenuPanel(cv::Mat& img, int img_width, int img_height)
{
    const int panel_width = 500;
    const int panel_height = img_height - 110;
    const int panel_x = img_width - panel_width - 10;
    const int panel_y = 10;
    
    cv::Mat overlay = img.clone();
    cv::rectangle(overlay, cv::Point(panel_x, panel_y), 
                 cv::Point(panel_x + panel_width, panel_y + panel_height), 
                 cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.7, img, 0.3, 0, img);
    
    int y_offset = panel_y + 25;
    const int text_box_width = 180;
    const int text_box_height = 28;
    const int line_spacing = 45;
    
    cv::putText(img, "=== CONTROL PANEL ===", 
               cv::Point(panel_x + 20, y_offset), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    y_offset += 35;

    std::string mode_text = g_ui_state.use_segmented_clusters_mode ? 
                        "Mode: SEGMENTED" : "Mode: FINAL OBJECTS";
    cv::Scalar mode_color = g_ui_state.use_segmented_clusters_mode ? 
                        cv::Scalar(180, 100, 180) : cv::Scalar(100, 180, 100);

    if (!g_segmented_mode_btn) {
        g_segmented_mode_btn = new Button(panel_x + 20, y_offset, 220, 32, 
                                        mode_text, g_ui_state.use_segmented_clusters_mode);
    } else {
        g_segmented_mode_btn->rect.x = panel_x + 20;
        g_segmented_mode_btn->rect.y = y_offset;
        g_segmented_mode_btn->label = mode_text;
        g_segmented_mode_btn->is_active = g_ui_state.use_segmented_clusters_mode;
    }
    g_segmented_mode_btn->draw(img, mode_color);
    y_offset += 40;

    // Info text about current mode
    std::string mode_info = g_ui_state.use_segmented_clusters_mode ? 
                        "View: Clusters (no velocity)" : "View: Objects (with velocity)";
    cv::putText(img, mode_info, 
            cv::Point(panel_x + 20, y_offset), 
            cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    y_offset += 25;

    cv::line(img, cv::Point(panel_x + 20, y_offset), 
            cv::Point(panel_x + panel_width - 20, y_offset), 
            cv::Scalar(80, 80, 80), 1);
    y_offset += 20;
    
    // DBSCAN Section
    cv::putText(img, "DBSCAN Parameters:", 
               cv::Point(panel_x + 20, y_offset), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 200, 255), 1, cv::LINE_AA);
    y_offset += 30;
    
    if (!g_minpoints_textbox) {
        g_minpoints_textbox = new TextBox(panel_x + 20, y_offset, text_box_width, text_box_height, 
                                         "MIN_POINTS:", &g_ui_state.min_points_text);
    } else {
        g_minpoints_textbox->rect.x = panel_x + 20;
        g_minpoints_textbox->rect.y = y_offset;
    }
    g_minpoints_textbox->is_active = (g_ui_state.active_textbox == UIState::MIN_POINTS_BOX);
    g_minpoints_textbox->draw(img);
    y_offset += line_spacing;
    
    if (!g_epsilon_textbox) {
        g_epsilon_textbox = new TextBox(panel_x + 20, y_offset, text_box_width, text_box_height, 
                                       "EPSILON:", &g_ui_state.epsilon_text);
    } else {
        g_epsilon_textbox->rect.x = panel_x + 20;
        g_epsilon_textbox->rect.y = y_offset;
    }
    g_epsilon_textbox->is_active = (g_ui_state.active_textbox == UIState::EPSILON_BOX);
    g_epsilon_textbox->draw(img);
    y_offset += line_spacing;
    
    if (!g_apply_dbscan_btn) {
        g_apply_dbscan_btn = new Button(panel_x + 20, y_offset, 160, 30, "Apply DBSCAN");
    } else {
        g_apply_dbscan_btn->rect.x = panel_x + 20;
        g_apply_dbscan_btn->rect.y = y_offset;
    }
    g_apply_dbscan_btn->draw(img, cv::Scalar(50, 150, 50));
    y_offset += 40;
    
    std::ostringstream dbscan_vals;
    dbscan_vals << "Active: MIN=" << g_ui_state.temp_min_points 
              << ", EPS=" << std::fixed << std::setprecision(2) << g_ui_state.temp_epsilon;
    cv::putText(img, dbscan_vals.str(), 
               cv::Point(panel_x + 20, y_offset), 
               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    y_offset += 25;
    
    // Point Cloud Filter Section
    cv::line(img, cv::Point(panel_x + 20, y_offset), 
            cv::Point(panel_x + panel_width - 20, y_offset), 
            cv::Scalar(80, 80, 80), 1);
    y_offset += 20;
    
    cv::putText(img, "Point Cloud Filter:", 
               cv::Point(panel_x + 20, y_offset), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 200, 255), 1, cv::LINE_AA);
    y_offset += 30;
    
    if (!g_cutoff_textbox) {
        g_cutoff_textbox = new TextBox(panel_x + 20, y_offset, text_box_width, text_box_height, 
                                       "Y < CUTOFF:", &g_ui_state.cutoff_distance_text);
    } else {
        g_cutoff_textbox->rect.x = panel_x + 20;
        g_cutoff_textbox->rect.y = y_offset;
    }
    g_cutoff_textbox->is_active = (g_ui_state.active_textbox == UIState::CUTOFF_DISTANCE_BOX);
    g_cutoff_textbox->draw(img);
    y_offset += line_spacing;
    
    if (!g_zmin_textbox) {
        g_zmin_textbox = new TextBox(panel_x + 20, y_offset, text_box_width, text_box_height, 
                                     "Z_MIN:", &g_ui_state.z_min_text);
    } else {
        g_zmin_textbox->rect.x = panel_x + 20;
        g_zmin_textbox->rect.y = y_offset;
    }
    g_zmin_textbox->is_active = (g_ui_state.active_textbox == UIState::Z_MIN_BOX);
    g_zmin_textbox->draw(img);
    y_offset += line_spacing;
    
    if (!g_zmax_textbox) {
        g_zmax_textbox = new TextBox(panel_x + 20, y_offset, text_box_width, text_box_height, 
                                     "Z_MAX:", &g_ui_state.z_max_text);
    } else {
        g_zmax_textbox->rect.x = panel_x + 20;
        g_zmax_textbox->rect.y = y_offset;
    }
    g_zmax_textbox->is_active = (g_ui_state.active_textbox == UIState::Z_MAX_BOX);
    g_zmax_textbox->draw(img);
    y_offset += line_spacing;
    
    if (!g_apply_filter_btn) {
        g_apply_filter_btn = new Button(panel_x + 20, y_offset, 160, 30, "Apply Filter");
    } else {
        g_apply_filter_btn->rect.x = panel_x + 20;
        g_apply_filter_btn->rect.y = y_offset;
    }
    g_apply_filter_btn->draw(img, cv::Scalar(50, 150, 50));
    y_offset += 40;
    
    std::ostringstream filter_vals;
    filter_vals << "Active: Y<" << std::fixed << std::setprecision(1) << g_ui_state.temp_cutoff_distance
              << ", Z[" << g_ui_state.temp_z_min << "," << g_ui_state.temp_z_max << "]";
    cv::putText(img, filter_vals.str(), 
               cv::Point(panel_x + 20, y_offset), 
               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    y_offset += 25;
    
    // Logging Section
    cv::line(img, cv::Point(panel_x + 20, y_offset), 
            cv::Point(panel_x + panel_width - 20, y_offset), 
            cv::Scalar(80, 80, 80), 1);
    y_offset += 20;
    
    cv::putText(img, "Data Logging:", 
               cv::Point(panel_x + 20, y_offset), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 200, 255), 1, cv::LINE_AA);
    y_offset += 30;
    
    std::string log_btn_text = g_ui_state.is_logging ? "Stop Logging" : "Start Logging";
    if (!g_logging_btn) {
        g_logging_btn = new Button(panel_x + 20, y_offset, 140, 30, log_btn_text, g_ui_state.is_logging);
    } else {
        g_logging_btn->rect.x = panel_x + 20;
        g_logging_btn->rect.y = y_offset;
        g_logging_btn->label = log_btn_text;
        g_logging_btn->is_active = g_ui_state.is_logging;
    }
    g_logging_btn->draw(img, g_ui_state.is_logging ? cv::Scalar(180, 50, 50) : cv::Scalar(50, 150, 50));
    y_offset += 40;
    
    if (g_ui_state.is_logging) {
        std::ostringstream log_info;
        log_info << "Frames logged: " << g_ui_state.frame_counter;
        cv::putText(img, log_info.str(), 
                   cv::Point(panel_x + 20, y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 255, 100), 1, cv::LINE_AA);
        y_offset += 25;
    } else {
        y_offset += 5;
    }
    
    std::string filter_text = g_ui_state.use_logging_filter ? "Disable Log Filter" : "Enable Log Filter";
    if (!g_log_filter_btn) {
        g_log_filter_btn = new Button(panel_x + 20, y_offset, 160, 28, filter_text, g_ui_state.use_logging_filter);
    } else {
        g_log_filter_btn->rect.x = panel_x + 20;
        g_log_filter_btn->rect.y = y_offset;
        g_log_filter_btn->label = filter_text;
        g_log_filter_btn->is_active = g_ui_state.use_logging_filter;
    }
    g_log_filter_btn->draw(img, g_ui_state.use_logging_filter ? cv::Scalar(100, 100, 180) : cv::Scalar(70, 70, 70));
    y_offset += 40;
    
    if (y_offset < panel_y + panel_height - 50) {
        const int compact_width = 85;
        const int compact_height = 24;
        const int x_col1 = panel_x + 20;
        const int x_col2 = panel_x + 20 + compact_width + 10;
        
        cv::putText(img, "Log Range Filters:", 
                   cv::Point(panel_x + 20, y_offset), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
        y_offset += 25;
        
        // X range
        if (!g_log_xmin_textbox) {
            g_log_xmin_textbox = new TextBox(x_col1, y_offset, compact_width, compact_height, 
                                            "X_MIN:", &g_ui_state.log_x_min_text);
        } else {
            g_log_xmin_textbox->rect = cv::Rect(x_col1, y_offset, compact_width, compact_height);
        }
        g_log_xmin_textbox->is_active = (g_ui_state.active_textbox == UIState::LOG_X_MIN_BOX);
        g_log_xmin_textbox->draw(img);
        
        if (!g_log_xmax_textbox) {
            g_log_xmax_textbox = new TextBox(x_col2, y_offset, compact_width, compact_height, 
                                            "X_MAX:", &g_ui_state.log_x_max_text);
        } else {
            g_log_xmax_textbox->rect = cv::Rect(x_col2, y_offset, compact_width, compact_height);
        }
        g_log_xmax_textbox->is_active = (g_ui_state.active_textbox == UIState::LOG_X_MAX_BOX);
        g_log_xmax_textbox->draw(img);
        y_offset += 35;
        
        // Y range
        if (!g_log_ymin_textbox) {
            g_log_ymin_textbox = new TextBox(x_col1, y_offset, compact_width, compact_height, 
                                            "Y_MIN:", &g_ui_state.log_y_min_text);
        } else {
            g_log_ymin_textbox->rect = cv::Rect(x_col1, y_offset, compact_width, compact_height);
        }
        g_log_ymin_textbox->is_active = (g_ui_state.active_textbox == UIState::LOG_Y_MIN_BOX);
        g_log_ymin_textbox->draw(img);
        
        if (!g_log_ymax_textbox) {
            g_log_ymax_textbox = new TextBox(x_col2, y_offset, compact_width, compact_height, 
                                            "Y_MAX:", &g_ui_state.log_y_max_text);
        } else {
            g_log_ymax_textbox->rect = cv::Rect(x_col2, y_offset, compact_width, compact_height);
        }
        g_log_ymax_textbox->is_active = (g_ui_state.active_textbox == UIState::LOG_Y_MAX_BOX);
        g_log_ymax_textbox->draw(img);
        y_offset += 35;
        
        // Z range
        if (!g_log_zmin_textbox) {
            g_log_zmin_textbox = new TextBox(x_col1, y_offset, compact_width, compact_height, 
                                            "Z_MIN:", &g_ui_state.log_z_min_text);
        } else {
            g_log_zmin_textbox->rect = cv::Rect(x_col1, y_offset, compact_width, compact_height);
        }
        g_log_zmin_textbox->is_active = (g_ui_state.active_textbox == UIState::LOG_Z_MIN_BOX);
        g_log_zmin_textbox->draw(img);
        
        if (!g_log_zmax_textbox) {
            g_log_zmax_textbox = new TextBox(x_col2, y_offset, compact_width, compact_height, 
                                            "Z_MAX:", &g_ui_state.log_z_max_text);
        } else {
            g_log_zmax_textbox->rect = cv::Rect(x_col2, y_offset, compact_width, compact_height);
        }
        g_log_zmax_textbox->is_active = (g_ui_state.active_textbox == UIState::LOG_Z_MAX_BOX);
        g_log_zmax_textbox->draw(img);
    }
}

/*
Control the longitudinal and transverse slope of road infrastructure according to TCVN 13592:2022
https://sgtvt.hochiminhcity.gov.vn/Files/1032/01-TCVN%2013592_2022_%C4%90%C6%B0%E1%BB%9Dng%20%C4%91%C3%B4%20th%E1%BB%8B_Y%C3%AAu%20c%E1%BA%A7u%20thi%E1%BA%BFt%20k%E1%BA%BF%20(1).pdf
Slope (%) | Inclination Angle (°)
1% | 0.57°
2% | 1.15°
5% | 2.86°
10% | 5.71°
15% | 8.53°
*/
 
/**
 * @brief Solves a regression equation with 6 unknowns to fit a quadratic surface
 * Requires at least 6 points for the regression
 * Used to analyze the shape of point clusters for slope detection
 * @param points Vector of Points containing x, y, z coordinates
 * @return Coefficients of the quadratic surface equation
*/
Eigen::VectorXd quadratic_regression(const std::vector<Point>& points)
{
    int n = points.size();
    Eigen::MatrixXd A(n, 6);
    Eigen::VectorXd b(n);
 
    for (int i = 0; i < n; ++i) {
        float x = points[i].x;
        float y = points[i].y;
        float z = points[i].z;
        A(i, 0) = x * x;  // a * x^2
        A(i, 1) = y * y;  // b * y^2
        A(i, 2) = x * y;  // c * xy
        A(i, 3) = x;      // d * x
        A(i, 4) = y;      // e * y
        A(i, 5) = 1;      // f
        b(i) = z;
    }
 
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
    return coeffs;
}

/**
 * @brief Determines if a cluster of points represents a sloped surface based on TCVN 13592:2022 standards
 * Checks both the flatness of the surface and its angle relative to horizontal
 * @param coeffs Coefficients from quadratic regression
 * @param slope_threshold Maximum angle to consider as a slope (degrees)
 * @return true if the cluster represents a valid slope
*/
bool is_slope_cluster(const Eigen::VectorXd& coeffs, double slope_threshold)
{
    double a = coeffs[0];  // x^2 coefficient
    double b = coeffs[1];  // y^2 coefficient
    double c = coeffs[2];  // xy coefficient
    double d = coeffs[3];  // x coefficient
    double e = coeffs[4];  // y coefficient
 
    bool is_flat_enough = false;
    bool is_slope_like = false;
   
    // Control the convexity/concavity of the conic surface along the axes
    if ((std::abs(a) < 0.1) && (std::abs(b) < 0.1) && (std::abs(c) < 0.1) && (std::abs(d) < 0.1))
    {
        is_flat_enough = true;
    }
 
    // The angle formed by the sloped surface with the XY plane
    double slope_angle = std::atan(std::abs(e)) * 180.0 / M_PI;
 
    if ((slope_angle >= 0.0) && (slope_angle <= slope_threshold))
    {
        is_slope_like = true;
    }
 
    return is_flat_enough && is_slope_like;
}

/**
 * @brief Extracts and filters points from a PointCloud message
 * Filters points based on distance and height range
 * @param out_pointcloud Input PointCloud message
 * @return Vector of Points after filtering
*/
std::vector<Point> extract_points_from_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                              float cutoff_distance,
                                              float z_min,
                                              float z_max)
{
    /* Fields of message:
        Field: x
        Field: y
        Field: z
        Field: Range
        Field: Velocity
        Field: AzimuthAngle
        Field: ElevationAngle
    */
    
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_vel(*msg, "Velocity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_az(*msg, "AzimuthAngle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_el(*msg, "ElevationAngle");

    std::vector<Point> points_vector;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_vel, ++iter_az, ++iter_el)
    {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        float v = *iter_vel;
        float az = *iter_az;
        float el = *iter_el;

        if (z > z_min && z <= z_max && x <= cutoff_distance) {
            Point radar_point(-y, x, z, v, -1, -1, az, el);
            points_vector.push_back(radar_point);

            RCLCPP_DEBUG(rclcpp::get_logger(__FUNCTION__), "x: %.2f y: %.2f z: %.2f v: %.2f az: %.2f el: %.2f",
            x, y, z, v, az, el);
        }
    }

    return points_vector;
}

/**
 * @brief Groups points by their cluster IDs after DBSCAN clustering
 * @param points Vector of all points with assigned cluster IDs
 * @param detected_clusters Number of clusters found
 * @return Vector of vectors, each containing points of one cluster
*/
std::vector<std::vector<Point>> segment_clusters_by_id(const std::vector<Point>& points, int detected_clusters)
{
    std::vector<std::vector<Point>> segmented_clusters(detected_clusters + 1);
    for (const Point& point : points)
    {
        if (point.clusterID != -1)
        {
            segmented_clusters.at(point.clusterID).push_back(point);
        }
    }
    return segmented_clusters;
}

/**
 * @brief Identifies clusters that are near the vehicle's position
 * Uses 1-meter threshold to determine proximity
 * @param segmented_clusters Vector of all clusters
 * @return Vector of cluster IDs that are near the origin
*/
std::vector<int> find_near_origin_cluster_ids(const std::vector<std::vector<Point>>& segmented_clusters)
{
    std::vector<int> near_origin_cluster_ids;
    for (const auto& cluster_vector : segmented_clusters)
    {
        bool has_near_origin_point = false;
        for (const Point& point : cluster_vector)
        {
            if (std::abs(point.x) < 6.0)  // Check if within n m of origin
            {
                has_near_origin_point = true;
                break;
            }
        }
        if (has_near_origin_point && !cluster_vector.empty())
        {
            near_origin_cluster_ids.push_back(cluster_vector[0].clusterID);
        }
    }
    return near_origin_cluster_ids;
}

/**
 * @brief Identifies and marks clusters that represent road slopes
 * Uses quadratic regression to analyze cluster geometry
 * @param segmented_clusters Vector of clusters to analyze
 * @param near_origin_cluster_ids IDs of clusters close to vehicle
*/
void mark_slope_clusters(std::vector<std::vector<Point>>& segmented_clusters, const std::vector<int>& near_origin_cluster_ids)
{
    for (auto& cluster_vector : segmented_clusters)
    {
        if (!cluster_vector.empty() &&
        std::find(near_origin_cluster_ids.begin(), near_origin_cluster_ids.end(),
        cluster_vector[0].clusterID) != near_origin_cluster_ids.end())
        {
            Eigen::VectorXd coeffs = quadratic_regression(cluster_vector);
            if (is_slope_cluster(coeffs))
            {
                for (auto& point : cluster_vector)
                {
                    point.clusterID = -99; // Mark as slope
                }
            }
        }
    }
}

/**
 * @brief Convert set of custom point to pcl::PointXYZ
 * @param cloud A Pointer to the point cloud
 * @return PointXYZ cloud
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(pcl::PointCloud<PointXYZV>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    output_cloud->header = cloud->header;
    output_cloud->width = cloud->width;
    output_cloud->height = cloud->height;
    output_cloud->is_dense = cloud->is_dense;
    output_cloud->points.reserve(cloud->points.size());

    for (const auto& pt : cloud->points) {
        pcl::PointXYZ xyz_point;
        xyz_point.x = pt.x;
        xyz_point.y = pt.y;
        xyz_point.z = pt.z;
        output_cloud->points.push_back(xyz_point);
    }

    return output_cloud;
}

/**
 * @brief Extractvelocity of a point cloud - a cluster
 * @param cloud A Pointer to the point cloud
 * @return V_x and V_y velocity 
*/
Eigen::Vector3f extract_velocity(const pcl::PointCloud<PointXYZV>::Ptr &cloud) {
    Eigen::MatrixXf A(cloud->points.size(), 3);
    Eigen::VectorXf V_r(cloud->points.size());

    // Build the linear system
    double theta;
    double elevate;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        theta = cloud->points[i].azimuth;
        elevate = cloud->points[i].elevate;
        A(i, 0) = std::cos(theta)*std::cos(elevate);
        A(i, 1) = std::sin(theta)*std::cos(elevate);
        A(i, 2) = std::sin(elevate);
        V_r(i) = cloud->points[i].velocity;
    }

    // Solve using pseudoinverse
    Eigen::Matrix3f ATA = A.transpose() * A;
    Eigen::MatrixXf A_pseudo = ATA.inverse() * A.transpose();
    Eigen::VectorXf V_i = A_pseudo * V_r;
    return Eigen::Vector3f(V_i(0), V_i(1), V_i(2)); // Return velocity vector [v_x, v_y]
}

/**
 * @brief Visualizes radar objects in a 2D top-down view optimized for Full HD display
 * 
 * Display specifications:
 * - Resolution: 1920x1080 (Full HD)
 * - Range: Y: 0~260m forward, X: -50~+50m lateral
 * - Coordinate: Y-up (forward), X-right, origin at bottom center
 * 
 * TUNING PARAMETERS:
 * - ORIGIN_OFFSET_FROM_BOTTOM: Origin offset from bottom edge (pixels)
 * - VELOCITY_ARROW_TIME_SCALE: Velocity arrow time scale (seconds)
 * - VELOCITY_ARROW_THICKNESS: Velocity arrow thickness (pixels)
 * - VELOCITY_ARROW_TIP_LENGTH: Velocity arrow tip length ratio
 * - MIN_VELOCITY_THRESHOLD: Minimum velocity to display (m/s)
 * - GRID_SPACING: Grid spacing (meters)
 * - GRID_MAJOR_INTERVAL: Major grid line interval (meters)
 */

// ========== TUNING PARAMETERS ==========
const int ORIGIN_OFFSET_FROM_BOTTOM = 50;
const float VELOCITY_ARROW_TIME_SCALE = 1.0f;
const int VELOCITY_ARROW_THICKNESS = 2;
const float VELOCITY_ARROW_TIP_LENGTH = 0.05f;
const float MIN_VELOCITY_THRESHOLD = 0.5f;
const int GRID_SPACING = 10;
const int GRID_MAJOR_INTERVAL = 50;

void visualizeRadarObjects2D(
    const std::vector<RadarObject>& objects,
    int max_range_y,
    int max_range_x,
    bool show_velocity,
    bool show_ids,
    bool show_grid,
    bool show_info,
    const std::string& window_name)
{
    // Full HD resolution
    const int img_width = 1920;
    const int img_height = 1080;
    
    // Calculate uniform scale that fills the screen for the specified range
    // Use MINIMUM scale to ensure BOTH axes fit on screen while keeping 1:1 aspect ratio
    float scale_x_candidate = img_width / (2.0f * max_range_x);
    float scale_y_candidate = (img_height - ORIGIN_OFFSET_FROM_BOTTOM) / max_range_y;

    // Use the SMALLER scale to ensure both ranges fit on screen
    const float scale = std::min(scale_x_candidate, scale_y_candidate);

    // Calculate actual display range based on the chosen scale
    // This will be equal to or larger than requested range
    float display_range_x = (img_width / scale) / 2.0f;
    float display_range_y = (img_height - ORIGIN_OFFSET_FROM_BOTTOM) / scale;

    // Create dark background
    cv::Mat img(img_height, img_width, CV_8UC3, cv::Scalar(20, 20, 20));

    // Origin position
    const int origin_x = img_width / 2;
    const int origin_y = img_height - ORIGIN_OFFSET_FROM_BOTTOM;

    // ========== DRAW GRID ==========
    g_mouse_data.origin_x = origin_x;
    g_mouse_data.origin_y = origin_y;
    g_mouse_data.scale = scale;

    if (show_grid)
    {
        cv::Scalar grid_color(60, 60, 60);
        cv::Scalar grid_color_major(100, 100, 100);
        
        // Adaptive grid spacing based on display range
        int adaptive_major_interval;
        if (display_range_y <= 30 || display_range_x <= 30) {
            adaptive_major_interval = 10;  // 10m for small ranges
        } else if (display_range_y <= 100 || display_range_x <= 100) {
            adaptive_major_interval = 20;  // 20m for medium ranges
        } else {
            adaptive_major_interval = 50;  // 50m for large ranges
        }
        
        // Vertical lines (X-axis, every GRID_SPACING meters)
        // Draw from center (0) outward to ensure alignment
        for (int x_m = 0; x_m <= static_cast<int>(display_range_x); x_m += GRID_SPACING)
        {
            bool is_major = (x_m % adaptive_major_interval == 0);
            cv::Scalar color = is_major ? grid_color_major : grid_color;
            int thickness = is_major ? 2 : 1;
            
            // Positive X
            int x_px_pos = origin_x + static_cast<int>(x_m * scale);
            if (x_px_pos >= 0 && x_px_pos < img_width)
            {
                cv::line(img, cv::Point(x_px_pos, 0), cv::Point(x_px_pos, img_height), color, thickness);
                
                // Label major lines (place above origin line with background)
                if (is_major && x_m != 0)
                {
                    std::string label = std::to_string(x_m) + "m";
                    int label_x = x_px_pos + 3;
                    int label_y = origin_y + 20;
                    
                    // Draw text background for better visibility
                    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                    cv::rectangle(img, 
                                cv::Point(label_x - 2, label_y - text_size.height - 2),
                                cv::Point(label_x + text_size.width + 2, label_y + 2),
                                cv::Scalar(0, 0, 0), -1);
                    
                    cv::putText(img, label, cv::Point(label_x, label_y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                }
            }
            
            // Negative X (mirror)
            if (x_m != 0)
            {
                int x_px_neg = origin_x - static_cast<int>(x_m * scale);
                if (x_px_neg >= 0 && x_px_neg < img_width)
                {
                    cv::line(img, cv::Point(x_px_neg, 0), cv::Point(x_px_neg, img_height), color, thickness);
                    
                    // Label major lines (place above origin line with background)
                    if (is_major)
                    {
                        std::string label = std::to_string(-x_m) + "m";
                        int label_x = x_px_neg + 3;
                        int label_y = origin_y + 20;
                        
                        // Draw text background for better visibility
                        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                        cv::rectangle(img, 
                                    cv::Point(label_x - 2, label_y - text_size.height - 2),
                                    cv::Point(label_x + text_size.width + 2, label_y + 2),
                                    cv::Scalar(0, 0, 0), -1);
                        
                        cv::putText(img, label, cv::Point(label_x, label_y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                    }
                }
            }
        }
        
        // Horizontal lines (Y-axis, every GRID_SPACING meters)
        for (int y_m = 0; y_m <= static_cast<int>(display_range_y); y_m += GRID_SPACING)
        {
            int y_px = origin_y - static_cast<int>(y_m * scale);
            if (y_px < 0) break;
            
            bool is_major = (y_m % adaptive_major_interval == 0);
            cv::Scalar color = is_major ? grid_color_major : grid_color;
            int thickness = is_major ? 2 : 1;
            cv::line(img, cv::Point(0, y_px), cv::Point(img_width, y_px), color, thickness);
            
            // Label major lines on the right side with background
            if (is_major && y_m != 0)
            {
                std::string label = std::to_string(y_m) + "m";
                cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                int label_x = img_width - text_size.width - 10;
                int label_y = y_px - 3;
                
                // Draw text background for better visibility
                cv::rectangle(img, 
                            cv::Point(label_x - 2, label_y - text_size.height - 2),
                            cv::Point(label_x + text_size.width + 2, label_y + 2),
                            cv::Scalar(0, 0, 0), -1);
                
                cv::putText(img, label, cv::Point(label_x, label_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }
        
        // Center line (X=0) - draw last so it's on top
        cv::line(img, cv::Point(origin_x, 0), cv::Point(origin_x, img_height), 
                cv::Scalar(150, 150, 0), 2, cv::LINE_AA);
    }

    // ========== DRAW RADAR OBJECTS ==========
    int object_count = 0;

    for (size_t i = 0; i < objects.size(); i++)
    {
        const auto& obj = objects[i];
        
        // Skip objects outside display range
        if (obj.y < 0 || obj.y > display_range_y || std::abs(obj.x) > display_range_x)
            continue;
        
        object_count++;
        
        // Generate distinct color for each object
        cv::Scalar color(
            static_cast<int>((i * 67 + 100) % 156 + 100),
            static_cast<int>((i * 157 + 50) % 156 + 100),
            static_cast<int>((i * 211 + 150) % 156 + 100)
        );
        
        // Draw point cloud
        for (const auto& pt : obj.point_cloud->points)
        {
            if (pt.y < 0 || pt.y > display_range_y || std::abs(pt.x) > display_range_x)
                continue;
                
            int u = origin_x + static_cast<int>(pt.x * scale);
            int v = origin_y - static_cast<int>(pt.y * scale);
            
            if (u >= 0 && u < img_width && v >= 0 && v < img_height)
            {
                cv::circle(img, cv::Point(u, v), 2, color, -1, cv::LINE_AA);
            }
        }
        
        // Draw bounding box
        int u_min = origin_x + static_cast<int>(obj.min_point.x * scale);
        int v_min = origin_y - static_cast<int>(obj.max_point.y * scale);
        int u_max = origin_x + static_cast<int>(obj.max_point.x * scale);
        int v_max = origin_y - static_cast<int>(obj.min_point.y * scale);
        
        cv::rectangle(img, cv::Point(u_min, v_min), cv::Point(u_max, v_max), 
                    color, 2, cv::LINE_AA);
        
        // Draw centroid
        int u_center = origin_x + static_cast<int>(obj.x * scale);
        int v_center = origin_y - static_cast<int>(obj.y * scale);
        cv::drawMarker(img, cv::Point(u_center, v_center), 
                    cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 8, 2, cv::LINE_AA);
        
        // Draw closest point marker
        int u_closest = origin_x + static_cast<int>(obj.closest_point.x * scale);
        int v_closest = origin_y - static_cast<int>(obj.closest_point.y * scale);
        
        if (u_closest >= 0 && u_closest < img_width && v_closest >= 0 && v_closest < img_height)
        {
            cv::drawMarker(img, cv::Point(u_closest, v_closest), 
                        cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS, 10, 1, cv::LINE_AA);
        }
        
        // Draw velocity arrow
        if (show_velocity)
        {
            float vel_magnitude = std::sqrt(obj.v_x * obj.v_x + obj.v_y * obj.v_y);
            if (vel_magnitude > MIN_VELOCITY_THRESHOLD)
            {
                int u_vel = u_center + static_cast<int>(obj.v_x * scale * VELOCITY_ARROW_TIME_SCALE);
                int v_vel = v_center - static_cast<int>(obj.v_y * scale * VELOCITY_ARROW_TIME_SCALE);
                
                cv::arrowedLine(img, cv::Point(u_center, v_center), 
                            cv::Point(u_vel, v_vel), 
                            cv::Scalar(0, 255, 0), 
                            VELOCITY_ARROW_THICKNESS,
                            cv::LINE_AA, 0, 
                            VELOCITY_ARROW_TIP_LENGTH);
                
                // Velocity text (convert m/s to km/h)
                float vel_kmh = vel_magnitude * 3.6f;
                std::ostringstream vel_text;
                vel_text << std::fixed << std::setprecision(1) << vel_kmh << " km/h";
                cv::putText(img, vel_text.str(), 
                        cv::Point(u_vel + 5, v_vel - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }
        
        // Draw object ID and info
        if (show_ids)
        {
            std::ostringstream info;
            info << "#" << i;
            
            // Background for text
            cv::Size text_size = cv::getTextSize(info.str(), cv::FONT_HERSHEY_SIMPLEX, 
                                                0.5, 2, nullptr);
            cv::rectangle(img, 
                        cv::Point(u_center - text_size.width / 2 - 3, v_center - text_size.height - 15),
                        cv::Point(u_center + text_size.width / 2 + 3, v_center - 10),
                        cv::Scalar(0, 0, 0), -1);
            
            cv::putText(img, info.str(), 
                    cv::Point(u_center - text_size.width / 2, v_center - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
        }
    }

    // ========== DRAW COORDINATE AXES ==========
    int axis_margin_x = img_width - 150;
    int axis_margin_y = 80;

        // Y-axis (Forward) - Green
    cv::arrowedLine(img, 
                cv::Point(axis_margin_x, axis_margin_y + 30),
                cv::Point(axis_margin_x, axis_margin_y - 30),
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.2);
    cv::putText(img, "Y", 
            cv::Point(axis_margin_x - 5, axis_margin_y - 40),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    cv::arrowedLine(img, 
                cv::Point(axis_margin_x, axis_margin_y),
                cv::Point(axis_margin_x + 60, axis_margin_y),
                cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.2);
    cv::putText(img, "X", 
            cv::Point(axis_margin_x + 65, axis_margin_y + 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        // Origin marker
    cv::circle(img, cv::Point(origin_x, origin_y), 5, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

        // ========== DRAW INFO OVERLAY ==========
    if (show_info)
    {
        // Fixed background height to avoid flickering when object count changes
        int info_panel_height = img_height - 40;  // Almost full height
        int info_panel_width = 450;
        
        // Semi-transparent background - draw once with fixed size
        cv::Mat overlay = img.clone();
        cv::rectangle(overlay, cv::Point(10, 10), cv::Point(info_panel_width, info_panel_height), 
                    cv::Scalar(0, 0, 0), -1);
        cv::addWeighted(overlay, 0.5, img, 0.5, 0, img);
        
        int text_y = 35;
        int line_height = 25;
        
        cv::putText(img, "=== 3D RADAR VIEW ===", 
                cv::Point(20, text_y), cv::FONT_HERSHEY_SIMPLEX, 
                0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        text_y += line_height;
        
        std::ostringstream info;
        // Display: visible objects in current view / total detected objects in frame
        info << "Objects: " << object_count << " / " << objects.size();
        cv::putText(img, info.str(), cv::Point(20, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        text_y += line_height;
        
        info.str("");
        info << "Focus range: Y=" << max_range_y << "m, X=+/-" << max_range_x << "m";
        cv::putText(img, info.str(), cv::Point(20, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        text_y += line_height;
        
        info.str("");
        info << "Actual view: Y=" << std::fixed << std::setprecision(1) 
            << display_range_y << "m, X=+/-" << display_range_x << "m";
        cv::putText(img, info.str(), cv::Point(20, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        text_y += line_height;
        
        info.str("");
        info << "Scale: " << std::fixed << std::setprecision(2) 
            << scale << " px/m (uniform)";
        cv::putText(img, info.str(), cv::Point(20, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        text_y += line_height;
        
        cv::putText(img, "Legend: Green=Velocity, Yellow=Centroid/Closest, Cyan=Origin", 
                cv::Point(20, text_y), cv::FONT_HERSHEY_SIMPLEX, 
                0.4, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
        text_y += line_height;
        
        // Object list
        cv::putText(img, "--- Objects ---", 
                cv::Point(20, text_y), cv::FONT_HERSHEY_SIMPLEX, 
                0.5, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        text_y += 20;
        
        for (size_t i = 0; i < objects.size(); i++)
        {
            const auto& obj = objects[i];
            
            // Skip objects outside display range
            if (obj.y < 0 || obj.y > display_range_y || std::abs(obj.x) > display_range_x)
                continue;
            
            // Generate same color as used for visualization
            cv::Scalar obj_color(
                static_cast<int>((i * 67 + 100) % 156 + 100),
                static_cast<int>((i * 157 + 50) % 156 + 100),
                static_cast<int>((i * 211 + 150) % 156 + 100)
            );
            
            // Draw color indicator
            cv::circle(img, cv::Point(30, text_y - 5), 4, obj_color, -1, cv::LINE_AA);
            
            // Draw object info
            info.str("");
            info << "#" << i << " - " << obj.point_cloud->points.size() << " pts";
            cv::putText(img, info.str(), cv::Point(45, text_y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
            
            text_y += 20;
            
            // Stop if reaching bottom of info panel
            if (text_y > info_panel_height - 30)
            {
                cv::putText(img, "...", cv::Point(30, text_y), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
                break;
            }
        }
    }

    int btn_width = 100;
    int btn_height = 35;
    int btn_x = img_width - btn_width - 20;
    int btn_y = img_height - btn_height - 20;
    
    if (!g_toggle_menu_btn) {
        g_toggle_menu_btn = new Button(btn_x, btn_y, btn_width, btn_height, 
                                       g_ui_state.show_menu ? "Hide Menu" : "Show Menu");
    } else {
        g_toggle_menu_btn->rect.x = btn_x;
        g_toggle_menu_btn->rect.y = btn_y;
        g_toggle_menu_btn->label = g_ui_state.show_menu ? "Hide Menu" : "Show Menu";
    }
    g_toggle_menu_btn->draw(img);

    if (g_ui_state.show_menu) {
        drawMenuPanel(img, img_width, img_height);
    }

    drawCrosshair(img, g_mouse_data, img_width, img_height);

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 1920, 1080);
    cv::setMouseCallback(window_name, onMouseEnhanced, nullptr);
    cv::imshow(window_name, img);
    
    int key = cv::waitKey(1);
    if (key != -1) {
        handleKeyboardInput(key);
    }
}

void visualizeSegmentedClusters(
    const std::vector<std::vector<Point>>& segmented_clusters,
    int max_range_y,
    int max_range_x,
    bool show_cluster_ids,
    bool show_grid,
    bool show_info,
    const std::string& window_name)
{
    const int img_width = 1920;
    const int img_height = 1080;
    
    float scale_x_candidate = img_width / (2.0f * max_range_x);
    float scale_y_candidate = (img_height - ORIGIN_OFFSET_FROM_BOTTOM) / max_range_y;
    const float scale = std::min(scale_x_candidate, scale_y_candidate);

    float display_range_x = (img_width / scale) / 2.0f;
    float display_range_y = (img_height - ORIGIN_OFFSET_FROM_BOTTOM) / scale;

    cv::Mat img(img_height, img_width, CV_8UC3, cv::Scalar(20, 20, 20));

    const int origin_x = img_width / 2;
    const int origin_y = img_height - ORIGIN_OFFSET_FROM_BOTTOM;

    g_mouse_data.origin_x = origin_x;
    g_mouse_data.origin_y = origin_y;
    g_mouse_data.scale = scale;

    // ========== DRAW GRID ==========
    if (show_grid)
    {
        cv::Scalar grid_color(60, 60, 60);
        cv::Scalar grid_color_major(100, 100, 100);
        
        int adaptive_major_interval;
        if (display_range_y <= 30 || display_range_x <= 30) {
            adaptive_major_interval = 10;
        } else if (display_range_y <= 100 || display_range_x <= 100) {
            adaptive_major_interval = 20;
        } else {
            adaptive_major_interval = 50;
        }
        
        // Vertical lines
        for (int x_m = 0; x_m <= static_cast<int>(display_range_x); x_m += GRID_SPACING)
        {
            bool is_major = (x_m % adaptive_major_interval == 0);
            cv::Scalar color = is_major ? grid_color_major : grid_color;
            int thickness = is_major ? 2 : 1;
            
            int x_px_pos = origin_x + static_cast<int>(x_m * scale);
            if (x_px_pos >= 0 && x_px_pos < img_width)
            {
                cv::line(img, cv::Point(x_px_pos, 0), cv::Point(x_px_pos, img_height), color, thickness);
                if (is_major && x_m != 0)
                {
                    std::string label = std::to_string(x_m) + "m";
                    int label_x = x_px_pos + 3;
                    int label_y = origin_y + 20;
                    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                    cv::rectangle(img, 
                                cv::Point(label_x - 2, label_y - text_size.height - 2),
                                cv::Point(label_x + text_size.width + 2, label_y + 2),
                                cv::Scalar(0, 0, 0), -1);
                    cv::putText(img, label, cv::Point(label_x, label_y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                }
            }
            
            if (x_m != 0)
            {
                int x_px_neg = origin_x - static_cast<int>(x_m * scale);
                if (x_px_neg >= 0 && x_px_neg < img_width)
                {
                    cv::line(img, cv::Point(x_px_neg, 0), cv::Point(x_px_neg, img_height), color, thickness);
                    if (is_major)
                    {
                        std::string label = std::to_string(-x_m) + "m";
                        int label_x = x_px_neg + 3;
                        int label_y = origin_y + 20;
                        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                        cv::rectangle(img, 
                                    cv::Point(label_x - 2, label_y - text_size.height - 2),
                                    cv::Point(label_x + text_size.width + 2, label_y + 2),
                                    cv::Scalar(0, 0, 0), -1);
                        cv::putText(img, label, cv::Point(label_x, label_y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                    }
                }
            }
        }
        
        // Horizontal lines
        for (int y_m = 0; y_m <= static_cast<int>(display_range_y); y_m += GRID_SPACING)
        {
            int y_px = origin_y - static_cast<int>(y_m * scale);
            if (y_px < 0) break;
            
            bool is_major = (y_m % adaptive_major_interval == 0);
            cv::Scalar color = is_major ? grid_color_major : grid_color;
            int thickness = is_major ? 2 : 1;
            cv::line(img, cv::Point(0, y_px), cv::Point(img_width, y_px), color, thickness);
            
            if (is_major && y_m != 0)
            {
                std::string label = std::to_string(y_m) + "m";
                cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                int label_x = img_width - text_size.width - 10;
                int label_y = y_px - 3;
                cv::rectangle(img, 
                            cv::Point(label_x - 2, label_y - text_size.height - 2),
                            cv::Point(label_x + text_size.width + 2, label_y + 2),
                            cv::Scalar(0, 0, 0), -1);
                cv::putText(img, label, cv::Point(label_x, label_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }
        
        cv::line(img, cv::Point(origin_x, 0), cv::Point(origin_x, img_height), 
                cv::Scalar(150, 150, 0), 2, cv::LINE_AA);
    }

    // ========== DRAW SEGMENTED CLUSTERS ==========
    int visible_cluster_count = 0;
    int total_clusters = 0;

    for (size_t i = 0; i < segmented_clusters.size(); i++)
    {
        const auto& cluster = segmented_clusters[i];
        
        if (cluster.empty() || cluster[0].clusterID == -99)
            continue;
        
        total_clusters++;
        int cluster_id = cluster[0].clusterID;
        
        // Calculate centroid and bounds
        float sum_x = 0, sum_y = 0, sum_z = 0;
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        float min_distance = std::numeric_limits<float>::max();
        Point closest_pt = cluster[0];
        
        for (const auto& pt : cluster)
        {
            sum_x += pt.x;
            sum_y += pt.y;
            sum_z += pt.z;
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
            
            float dist = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            if (dist < min_distance) {
                min_distance = dist;
                closest_pt = pt;
            }
        }
        
        float centroid_x = sum_x / cluster.size();
        float centroid_y = sum_y / cluster.size();
        
        if (centroid_y < 0 || centroid_y > display_range_y || 
            std::abs(centroid_x) > display_range_x)
            continue;
        
        visible_cluster_count++;
        
        // Generate color based on cluster ID
        cv::Scalar color(
            static_cast<int>((cluster_id * 67 + 100) % 156 + 100),
            static_cast<int>((cluster_id * 157 + 50) % 156 + 100),
            static_cast<int>((cluster_id * 211 + 150) % 156 + 100)
        );
        
        // Draw points
        for (const auto& pt : cluster)
        {
            if (pt.y < 0 || pt.y > display_range_y || std::abs(pt.x) > display_range_x)
                continue;
                
            int u = origin_x + static_cast<int>(pt.x * scale);
            int v = origin_y - static_cast<int>(pt.y * scale);
            
            if (u >= 0 && u < img_width && v >= 0 && v < img_height)
            {
                cv::circle(img, cv::Point(u, v), 2, color, -1, cv::LINE_AA);
            }
        }
        
        // Draw bounding box
        int u_min = origin_x + static_cast<int>(min_x * scale);
        int v_min = origin_y - static_cast<int>(max_y * scale);
        int u_max = origin_x + static_cast<int>(max_x * scale);
        int v_max = origin_y - static_cast<int>(min_y * scale);
        cv::rectangle(img, cv::Point(u_min, v_min), cv::Point(u_max, v_max), 
                    color, 2, cv::LINE_AA);
        
        // Draw centroid
        int u_center = origin_x + static_cast<int>(centroid_x * scale);
        int v_center = origin_y - static_cast<int>(centroid_y * scale);
        cv::drawMarker(img, cv::Point(u_center, v_center), 
                    cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 8, 2, cv::LINE_AA);
        
        // Draw closest point
        int u_closest = origin_x + static_cast<int>(closest_pt.x * scale);
        int v_closest = origin_y - static_cast<int>(closest_pt.y * scale);
        if (u_closest >= 0 && u_closest < img_width && v_closest >= 0 && v_closest < img_height)
        {
            cv::drawMarker(img, cv::Point(u_closest, v_closest), 
                        cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS, 10, 1, cv::LINE_AA);
        }
        
        // Draw cluster ID
        if (show_cluster_ids)
        {
            std::ostringstream info;
            info << "ID:" << cluster_id;
            
            cv::Size text_size = cv::getTextSize(info.str(), cv::FONT_HERSHEY_SIMPLEX, 
                                                0.5, 2, nullptr);
            cv::rectangle(img, 
                        cv::Point(u_center - text_size.width / 2 - 3, v_center - text_size.height - 15),
                        cv::Point(u_center + text_size.width / 2 + 3, v_center - 10),
                        cv::Scalar(0, 0, 0), -1);
            
            cv::putText(img, info.str(), 
                    cv::Point(u_center - text_size.width / 2, v_center - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
        }
    }

    // ========== DRAW AXES ==========
    int axis_margin_x = img_width - 150;
    int axis_margin_y = 80;

    cv::arrowedLine(img, 
                cv::Point(axis_margin_x, axis_margin_y + 30),
                cv::Point(axis_margin_x, axis_margin_y - 30),
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.2);
    cv::putText(img, "Y", 
            cv::Point(axis_margin_x - 5, axis_margin_y - 40),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    cv::arrowedLine(img, 
                cv::Point(axis_margin_x, axis_margin_y),
                cv::Point(axis_margin_x + 60, axis_margin_y),
                cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.2);
    cv::putText(img, "X", 
            cv::Point(axis_margin_x + 65, axis_margin_y + 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    cv::circle(img, cv::Point(origin_x, origin_y), 5, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

    // ========== INFO PANEL ==========
    if (show_info)
    {
        int info_panel_height = img_height - 40;
        int info_panel_width = 450;
        
        cv::Mat overlay = img.clone();
        cv::rectangle(overlay, cv::Point(10, 10), cv::Point(info_panel_width, info_panel_height), 
                    cv::Scalar(0, 0, 0), -1);
        cv::addWeighted(overlay, 0.5, img, 0.5, 0, img);
        
        int text_y = 35;
        int line_height = 25;
        
        cv::putText(img, "=== SEGMENTED CLUSTERS ===", 
                cv::Point(20, text_y), cv::FONT_HERSHEY_SIMPLEX, 
                0.6, cv::Scalar(180, 100, 255), 2, cv::LINE_AA);
        text_y += line_height;
        
        std::ostringstream info;
        info << "Clusters: " << visible_cluster_count << " / " << total_clusters;
        cv::putText(img, info.str(), cv::Point(20, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        text_y += line_height;
        
        info.str("");
        info << "Focus range: Y=" << max_range_y << "m, X=+/-" << max_range_x << "m";
        cv::putText(img, info.str(), cv::Point(20, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        text_y += line_height;
        
        info.str("");
        info << "Actual view: Y=" << std::fixed << std::setprecision(1) 
            << display_range_y << "m, X=+/-" << display_range_x << "m";
        cv::putText(img, info.str(), cv::Point(20, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        text_y += line_height;
        
        cv::putText(img, "Mode: After preprocessing (no velocity)", 
                cv::Point(20, text_y), cv::FONT_HERSHEY_SIMPLEX, 
                0.4, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
        text_y += line_height;
        
        cv::putText(img, "Legend: Yellow=Centroid/Closest", 
                cv::Point(20, text_y), cv::FONT_HERSHEY_SIMPLEX, 
                0.4, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
        text_y += 30;
        
        cv::putText(img, "--- Clusters ---", 
                cv::Point(20, text_y), cv::FONT_HERSHEY_SIMPLEX, 
                0.5, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        text_y += 20;
        
        for (size_t i = 0; i < segmented_clusters.size(); i++)
        {
            const auto& cluster = segmented_clusters[i];
            
            if (cluster.empty() || cluster[0].clusterID == -99)
                continue;
            
            int cluster_id = cluster[0].clusterID;
            float centroid_x = 0, centroid_y = 0;
            for (const auto& pt : cluster) {
                centroid_x += pt.x;
                centroid_y += pt.y;
            }
            centroid_x /= cluster.size();
            centroid_y /= cluster.size();
            
            if (centroid_y < 0 || centroid_y > display_range_y || 
                std::abs(centroid_x) > display_range_x)
                continue;
            
            cv::Scalar obj_color(
                static_cast<int>((cluster_id * 67 + 100) % 156 + 100),
                static_cast<int>((cluster_id * 157 + 50) % 156 + 100),
                static_cast<int>((cluster_id * 211 + 150) % 156 + 100)
            );
            
            cv::circle(img, cv::Point(30, text_y - 5), 4, obj_color, -1, cv::LINE_AA);
            
            info.str("");
            info << "ID:" << cluster_id << " - " << cluster.size() << " pts";
            cv::putText(img, info.str(), cv::Point(45, text_y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
            
            text_y += 20;
            
            if (text_y > info_panel_height - 30)
            {
                cv::putText(img, "...", cv::Point(30, text_y), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
                break;
            }
        }
    }

    // Menu and crosshair
    int btn_width = 100;
    int btn_height = 35;
    int btn_x = img_width - btn_width - 20;
    int btn_y = img_height - btn_height - 20;
    
    if (!g_toggle_menu_btn) {
        g_toggle_menu_btn = new Button(btn_x, btn_y, btn_width, btn_height, 
                                       g_ui_state.show_menu ? "Hide Menu" : "Show Menu");
    } else {
        g_toggle_menu_btn->rect.x = btn_x;
        g_toggle_menu_btn->rect.y = btn_y;
        g_toggle_menu_btn->label = g_ui_state.show_menu ? "Hide Menu" : "Show Menu";
    }
    g_toggle_menu_btn->draw(img);

    if (g_ui_state.show_menu) {
        drawMenuPanel(img, img_width, img_height);
    }

    drawCrosshair(img, g_mouse_data, img_width, img_height);

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 1920, 1080);
    cv::setMouseCallback(window_name, onMouseEnhanced, nullptr);
    cv::imshow(window_name, img);
    
    int key = cv::waitKey(1);
    if (key != -1) {
        handleKeyboardInput(key);
    }
}

/**
 * @file velocity_filtering_simple.cpp
 * @brief Simple sequential velocity-based filtering for road contamination removal
 * 
 * Add this code to the END of utilities.cpp
 */

#include "utilities.h"
#include <algorithm>
#include <cmath>

// Check if a velocity is close to any velocity in a subcluster (within VELOCITY_EPS)
bool isVelocityCloseToSubcluster(float velocity, const std::vector<Point>& subcluster)
{
    for (const auto& pt : subcluster) {
        if (std::abs(velocity - pt.velocity) <= VELOCITY_EPS) {
            return true;
        }
    }
    return false;
}

bool filterRoadFromVehicleCluster(
    const std::vector<Point>& cluster,
    std::vector<Point>& filtered_cluster)
{
    filtered_cluster.clear();
    
    // Need at least 2 points to split
    if (cluster.size() < 2) {
        filtered_cluster = cluster;
        return false;
    }
    
    // Sequential velocity clustering into 3 subclusters
    std::vector<Point> subcluster1;
    std::vector<Point> subcluster2;
    std::vector<Point> subcluster3;
    
    // First point goes to subcluster 1
    subcluster1.push_back(cluster[0]);
    
    // Process remaining points sequentially
    for (size_t i = 1; i < cluster.size(); ++i) {
        const Point& pt = cluster[i];
        
        // Try to add to subcluster1 first
        if (isVelocityCloseToSubcluster(pt.velocity, subcluster1)) {
            subcluster1.push_back(pt);
        }
        // If not subcluster1, try subcluster2
        else if (!subcluster2.empty() && isVelocityCloseToSubcluster(pt.velocity, subcluster2)) {
            subcluster2.push_back(pt);
        }
        // If doesn't match subcluster1 or subcluster2, goes to subcluster3
        else {
            // If subcluster2 is still empty, this becomes first point of subcluster2
            if (subcluster2.empty()) {
                subcluster2.push_back(pt);
            }
            // If subcluster2 already has points, check subcluster3
            else if (!subcluster3.empty() && isVelocityCloseToSubcluster(pt.velocity, subcluster3)) {
                subcluster3.push_back(pt);
            }
            // First point of subcluster3
            else if (subcluster3.empty()) {
                subcluster3.push_back(pt);
            }
            // Doesn't match any existing subcluster, add to subcluster3 anyway
            else {
                subcluster3.push_back(pt);
            }
        }
    }
    
    // If all points went to one subcluster -> homogeneous, no split needed
    if (subcluster2.empty() && subcluster3.empty()) {
        filtered_cluster = cluster;
        return false;
    }
    
    // Calculate mean z and mean velocity for all non-empty subclusters
    struct SubclusterInfo {
        std::vector<Point>* points;
        float mean_z;
        float mean_vel;
        int id;
    };
    
    std::vector<SubclusterInfo> subclusters;
    
    // Process subcluster1
    if (!subcluster1.empty()) {
        SubclusterInfo info;
        info.points = &subcluster1;
        info.id = 1;
        
        info.mean_z = 0.0f;
        info.mean_vel = 0.0f;
        for (const auto& pt : subcluster1) {
            info.mean_z += pt.z;
            info.mean_vel += pt.velocity;
        }
        info.mean_z /= subcluster1.size();
        info.mean_vel /= subcluster1.size();
        
        subclusters.push_back(info);
    }
    
    // Process subcluster2
    if (!subcluster2.empty()) {
        SubclusterInfo info;
        info.points = &subcluster2;
        info.id = 2;
        
        info.mean_z = 0.0f;
        info.mean_vel = 0.0f;
        for (const auto& pt : subcluster2) {
            info.mean_z += pt.z;
            info.mean_vel += pt.velocity;
        }
        info.mean_z /= subcluster2.size();
        info.mean_vel /= subcluster2.size();
        
        subclusters.push_back(info);
    }
    
    // Process subcluster3
    if (!subcluster3.empty()) {
        SubclusterInfo info;
        info.points = &subcluster3;
        info.id = 3;
        
        info.mean_z = 0.0f;
        info.mean_vel = 0.0f;
        for (const auto& pt : subcluster3) {
            info.mean_z += pt.z;
            info.mean_vel += pt.velocity;
        }
        info.mean_z /= subcluster3.size();
        info.mean_vel /= subcluster3.size();
        
        subclusters.push_back(info);
    }
    
    // Find subcluster with highest mean_z (vehicle)
    SubclusterInfo* highest_z_cluster = &subclusters[0];
    for (size_t i = 1; i < subclusters.size(); ++i) {
        if (subclusters[i].mean_z > highest_z_cluster->mean_z) {
            highest_z_cluster = &subclusters[i];
        }
    }
    
    // Keep only the subcluster with highest z
    filtered_cluster = *(highest_z_cluster->points);
    
    return true;
}

void filterNearOriginClusters(
    std::vector<std::vector<Point>>& segmented_clusters,
    const std::vector<int>& near_origin_cluster_ids)
{
    // Process each cluster
    for (auto& cluster : segmented_clusters) {
        // Skip empty or slope clusters
        if (cluster.empty() || cluster[0].clusterID == -99) {
            continue;
        }
        
        // Check if this cluster is in near_origin list
        int cluster_id = cluster[0].clusterID;
        bool is_near_origin = (std::find(near_origin_cluster_ids.begin(), 
                                         near_origin_cluster_ids.end(), 
                                         cluster_id) != near_origin_cluster_ids.end());
        
        // Only filter near-origin clusters
        if (!is_near_origin) {
            continue;
        }
        
        // Apply velocity-based filtering
        std::vector<Point> filtered_cluster;
        bool was_filtered = filterRoadFromVehicleCluster(cluster, filtered_cluster);
        
        if (was_filtered) {
            cluster = filtered_cluster;  // Replace with filtered version
        }
    }
}