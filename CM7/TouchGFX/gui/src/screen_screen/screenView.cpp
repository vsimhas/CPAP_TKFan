/**
 * screenView.cpp - CPAP menu view (rendered entirely in user code).
 */
#include <gui/screen_screen/screenView.hpp>
#include <gui/cpap/cpap_menu.hpp>
#include <gui/cpap/cpap_settings.hpp>
#include <gui/cpap/status_icons.hpp>

#include <texts/TextKeysAndLanguages.hpp>
#include <touchgfx/Color.hpp>
#include <touchgfx/Unicode.hpp>

#include <string.h>
#include <stdio.h>

extern "C" {
    #include "rotary_input.h"
    #include "dbg_log.h"
    #include "diagnostics_data.h"
    #include "therapy.h"
    #include "epr_ctrl.h"
    #include "service_reminders.h"   /* #87 */
}

using namespace touchgfx;

namespace {

/* Palette (16bpp RGB565). Kept inline so a future theme swap is one place. */
const colortype kColBg        = Color::getColorFromRGB( 12,  18,  32); /* near-black blue   */
const colortype kColStatusBg  = Color::getColorFromRGB( 28,  40,  64);
const colortype kColTitleBg   = Color::getColorFromRGB( 18,  28,  48);
const colortype kColContentBg = Color::getColorFromRGB( 18,  28,  48);
const colortype kColRowSel    = Color::getColorFromRGB( 60, 140, 200); /* teal highlight    */
const colortype kColRowEdit   = Color::getColorFromRGB(200, 150,  50); /* amber for edit    */
const colortype kColText      = Color::getColorFromRGB(235, 240, 248);
const colortype kColTextDim   = Color::getColorFromRGB(160, 175, 200);
const colortype kColWave      = Color::getColorFromRGB(120, 200, 255);
/* Icon-specific colours so humidifier states are distinguishable at a
 * glance, even if the user can't read the glyph clearly at 16x16. */
const colortype kColIconWarn  = Color::getColorFromRGB(220,  60,  60); /* red    */
const colortype kColIconWarm  = Color::getColorFromRGB(220, 160,  40); /* amber  */
const colortype kColIconCool  = Color::getColorFromRGB( 90, 170, 240); /* blue   */
/* Diagnostics-page test-status colours. Reuse warm/warn/cool where they
 * already exist; only "PASS" needs a brand-new green entry. */
const colortype kColTestPass  = Color::getColorFromRGB( 90, 200, 110); /* green  */

constexpr int16_t kScreenW   = 480;
constexpr int16_t kScreenH   = 272;
constexpr int16_t kStatusH   = 32;
constexpr int16_t kTitleY    = 32;
constexpr int16_t kTitleH    = 38;
constexpr int16_t kContentY  = 70;
constexpr int16_t kContentH  = kScreenH - kContentY;
constexpr int16_t kRowH      = 28;
constexpr int16_t kRowsTopY  = kContentY + 4;

/* Copy an ASCII C string into a UnicodeChar buffer, clamped to buf_chars-1
 * plus a NUL terminator. */
void utf8ToBuf(const char* src, Unicode::UnicodeChar* dst, uint16_t buf_chars)
{
    if (src == nullptr) src = "";
    Unicode::strncpy(dst, src, buf_chars - 1U);
    dst[buf_chars - 1U] = 0;
}

} // namespace

screenView::screenView()
    : row_visible_count_(0U),
      list_scroll_offset_(0U),
      therapy_hist_count_(0U),
      therapy_hist_decim_(0U)
{
    for (int i = 0; i < kWaveBars; ++i)
    {
        therapy_pressure_hist_[i] = 0.0f;
    }
}

void screenView::setupScreen()
{
    screenViewBase::setupScreen();
    buildWidgets();
    renderCurrentPage();
    /* Note: Screen::handleTickEvent() is invoked automatically by the
     * Application on the active screen each frame, no registerTimerWidget()
     * needed (Screen is not a Drawable, so that API does not apply here). */
}

void screenView::tearDownScreen()
{
    screenViewBase::tearDownScreen();
}

void screenView::buildWidgets()
{
    /* Hide whatever the Designer base put on the screen (the puzzle image
     * and the test button). We add our own children below. */
    image1.setVisible(false);
    buttonWithLabel1.setVisible(false);
    /* The Designer base sized __background as 479x271 (off-by-one). Resize
     * it so the chosen colour fills the full panel. */
    __background.setPosition(0, 0, kScreenW, kScreenH);
    __background.setColor(kColBg);

    /* --------- Status bar (always visible) ------------------------- */
    status_bar_bg_.setPosition(0, 0, kScreenW, kStatusH);
    status_bar_bg_.setColor(kColStatusBg);
    add(status_bar_bg_);

    status_left_text_.setPosition(8, 6, 220, kStatusH - 12);
    status_left_text_.setColor(kColText);
    status_left_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("Home", status_left_buf_, sizeof(status_left_buf_) / sizeof(status_left_buf_[0]));
    status_left_text_.setWildcard(status_left_buf_);
    add(status_left_text_);

    /* --------- Status-bar icons (right side, right-anchored) ---------
     *
     * Layout: 16x16 icons spaced 20 px apart, vertically centred in the
     * 32-px status bar (y = 8). Slots from right edge inward:
     *   slot 0 (rightmost) : Home          (only on HOME page)
     *   slot 1             : Airplane mode (only when airplane=ON)
     *   slot 2             : Cellular      (bars 1..4 OR no-signal X)
     *   slot 3             : Bluetooth     (only when paired)
     *   slot 4             : Humidifier    (fault/warming/cooling)
     *
     * Each icon's mask + visibility is set in updateStatusBar(). They are
     * registered with add() unconditionally so we can flip visibility
     * cheaply without reflowing the parent. */
    const int16_t kIconY = 8;
    const int16_t kIconStep = 20;
    const int16_t kSlot0X = kScreenW - 16 - 8;     /* rightmost icon, 8 px right padding */

    icon_home_     .setXY(kSlot0X - 0 * kIconStep, kIconY);
    icon_airplane_ .setXY(kSlot0X - 1 * kIconStep, kIconY);
    icon_cellular_ .setXY(kSlot0X - 2 * kIconStep, kIconY);
    icon_bluetooth_.setXY(kSlot0X - 3 * kIconStep, kIconY);
    icon_humidifier_.setXY(kSlot0X - 4 * kIconStep, kIconY);

    icon_home_     .setIcon(cpap::kIconHome);
    icon_airplane_ .setIcon(cpap::kIconAirplane);
    icon_cellular_ .setIcon(cpap::kIconNoCellular);
    icon_bluetooth_.setIcon(cpap::kIconBluetooth);
    icon_humidifier_.setIcon(cpap::kIconHumFault);

    icon_home_     .setColor(kColText);
    icon_airplane_ .setColor(kColText);
    icon_cellular_ .setColor(kColText);
    icon_bluetooth_.setColor(kColText);
    icon_humidifier_.setColor(kColIconWarn);

    /* Start hidden; updateStatusBar() will turn the relevant ones on. */
    icon_home_     .setVisible(false);
    icon_airplane_ .setVisible(false);
    icon_cellular_ .setVisible(false);
    icon_bluetooth_.setVisible(false);
    icon_humidifier_.setVisible(false);

    add(icon_home_);
    add(icon_airplane_);
    add(icon_cellular_);
    add(icon_bluetooth_);
    add(icon_humidifier_);

    /* --------- Title bar ------------------------------------------- */
    title_bar_bg_.setPosition(0, kTitleY, kScreenW, kTitleH);
    title_bar_bg_.setColor(kColTitleBg);
    add(title_bar_bg_);

    title_text_.setPosition(16, kTitleY + 4, kScreenW - 32, kTitleH - 8);
    title_text_.setColor(kColText);
    title_text_.setTypedText(TypedText(T_DYN_M));
    utf8ToBuf("CPAP", title_buf_, sizeof(title_buf_) / sizeof(title_buf_[0]));
    title_text_.setWildcard(title_buf_);
    add(title_text_);

    /* --------- Content background --------------------------------- */
    content_bg_.setPosition(0, kContentY, kScreenW, kContentH);
    content_bg_.setColor(kColContentBg);
    add(content_bg_);

    /* --------- Menu rows ------------------------------------------ */
    for (int i = 0; i < kMaxRows; ++i)
    {
        int16_t y = kRowsTopY + (int16_t)(i * kRowH);

        row_bg_[i].setPosition(8, y, kScreenW - 16, kRowH - 2);
        row_bg_[i].setColor(kColContentBg);
        row_bg_[i].setVisible(false);
        add(row_bg_[i]);

        row_label_[i].setPosition(16, y + 4, 280, kRowH - 6);
        row_label_[i].setColor(kColText);
        row_label_[i].setTypedText(TypedText(T_DYN_M));
        utf8ToBuf("", row_label_buf_[i],
                  sizeof(row_label_buf_[i]) / sizeof(row_label_buf_[i][0]));
        row_label_[i].setWildcard(row_label_buf_[i]);
        row_label_[i].setVisible(false);
        add(row_label_[i]);

        row_value_[i].setPosition(300, y + 4, kScreenW - 316, kRowH - 6);
        row_value_[i].setColor(kColTextDim);
        row_value_[i].setTypedText(TypedText(T_DYN_M));
        utf8ToBuf("", row_value_buf_[i],
                  sizeof(row_value_buf_[i]) / sizeof(row_value_buf_[i][0]));
        row_value_[i].setWildcard(row_value_buf_[i]);
        row_value_[i].setVisible(false);
        add(row_value_[i]);
    }

    /* --------- Value-editor (LARGE centred) ------------------------ */
    editor_value_text_.setPosition(0, kContentY + 40, kScreenW, 60);
    editor_value_text_.setColor(kColRowEdit);
    editor_value_text_.setTypedText(TypedText(T_DYN_L_CENTER));
    utf8ToBuf("", editor_value_buf_,
              sizeof(editor_value_buf_) / sizeof(editor_value_buf_[0]));
    editor_value_text_.setWildcard(editor_value_buf_);
    editor_value_text_.setVisible(false);
    add(editor_value_text_);

    editor_hint_text_.setPosition(0, kScreenH - 28, kScreenW, 20);
    editor_hint_text_.setColor(kColTextDim);
    editor_hint_text_.setTypedText(TypedText(T_DYN_M_CENTER));
    utf8ToBuf("", editor_hint_buf_,
              sizeof(editor_hint_buf_) / sizeof(editor_hint_buf_[0]));
    editor_hint_text_.setWildcard(editor_hint_buf_);
    editor_hint_text_.setVisible(false);
    add(editor_hint_text_);

    /* --------- THERAPY: live pressure trace + texts ---------------- */
    {
        const int16_t bars_y    = kContentY + 30;
        const int16_t bars_h    = kTherapyGraphH;
        const int16_t bars_left = 20;
        const int16_t bars_w    = kScreenW - 2 * bars_left;
        const int16_t bar_w     = bars_w / kWaveBars;

        therapy_setpoint_line_.setPosition(bars_left, bars_y + bars_h / 2, bars_w, 2);
        therapy_setpoint_line_.setColor(kColRowEdit);
        therapy_setpoint_line_.setVisible(false);
        add(therapy_setpoint_line_);

        for (int i = 0; i < kWaveBars; ++i)
        {
            int16_t x = bars_left + (int16_t)(i * bar_w);
            wave_bars_[i].setPosition(x, bars_y + bars_h - 4, bar_w - 2, 4);
            wave_bars_[i].setColor(kColWave);
            wave_bars_[i].setVisible(false);
            add(wave_bars_[i]);
        }

        therapy_pressure_text_.setPosition(0, kContentY + 4, kScreenW, 26);
        therapy_pressure_text_.setColor(kColText);
        therapy_pressure_text_.setTypedText(TypedText(T_DYN_M_CENTER));
        utf8ToBuf("", therapy_pressure_buf_,
                  sizeof(therapy_pressure_buf_) / sizeof(therapy_pressure_buf_[0]));
        therapy_pressure_text_.setWildcard(therapy_pressure_buf_);
        therapy_pressure_text_.setVisible(false);
        add(therapy_pressure_text_);

        therapy_live_big_text_.setPosition(bars_left, bars_y + 8, bars_w, 56);
        therapy_live_big_text_.setColor(kColText);
        therapy_live_big_text_.setTypedText(TypedText(T_DYN_L_CENTER));
        utf8ToBuf("0.0", therapy_live_big_buf_,
                  sizeof(therapy_live_big_buf_) / sizeof(therapy_live_big_buf_[0]));
        therapy_live_big_text_.setWildcard(therapy_live_big_buf_);
        therapy_live_big_text_.setVisible(false);
        add(therapy_live_big_text_);

        therapy_summary_text_.setPosition(0, kScreenH - 30, kScreenW, 22);
        therapy_summary_text_.setColor(kColTextDim);
        therapy_summary_text_.setTypedText(TypedText(T_DYN_M_CENTER));
        utf8ToBuf("", therapy_summary_buf_,
                  sizeof(therapy_summary_buf_) / sizeof(therapy_summary_buf_[0]));
        therapy_summary_text_.setWildcard(therapy_summary_buf_);
        therapy_summary_text_.setVisible(false);
        add(therapy_summary_text_);
    }

    /* --------- WELCOME -------------------------------------------- */
    welcome_big_text_.setPosition(0, kContentY + 30, kScreenW, 60);
    welcome_big_text_.setColor(kColText);
    welcome_big_text_.setTypedText(TypedText(T_DYN_L_CENTER));
    utf8ToBuf("Welcome", welcome_big_buf_,
              sizeof(welcome_big_buf_) / sizeof(welcome_big_buf_[0]));
    welcome_big_text_.setWildcard(welcome_big_buf_);
    welcome_big_text_.setVisible(false);
    add(welcome_big_text_);

    welcome_sub_text_.setPosition(0, kContentY + 110, kScreenW, 26);
    welcome_sub_text_.setColor(kColTextDim);
    welcome_sub_text_.setTypedText(TypedText(T_DYN_M_CENTER));
    utf8ToBuf("Press the knob to begin",
              welcome_sub_buf_,
              sizeof(welcome_sub_buf_) / sizeof(welcome_sub_buf_[0]));
    welcome_sub_text_.setWildcard(welcome_sub_buf_);
    welcome_sub_text_.setVisible(false);
    add(welcome_sub_text_);

    /* --------- DIAGNOSTICS -----------------------------------------
     * Two columns, each with 1 header row + N data rows. We use the
     * "small" font (T_DYN_S = Verdana 10) so 8 rows fit comfortably
     * in the 202-px content area. Position offsets:
     *
     *   left  column : labels @ x=12,  values @ x=120, width 100
     *   right column : labels @ x=242, values @ x=350, width 110
     *
     * Index layout (kept in screenView.hpp):
     *   0     : left header     ("SELF-TESTS")
     *   1..4  : SD / NFC / SDRAM / QSPI rows
     *   5     : right header    ("LIVE SENSORS")
     *   6..12 : 7 sensor rows
     */
    {
        const int16_t kDiagTopY      = kContentY + 4;
        const int16_t kDiagLineH     = 18;
        const int16_t kDiagLeftLblX  = 12;
        const int16_t kDiagLeftValX  = 120;
        const int16_t kDiagLeftLblW  = 100;
        const int16_t kDiagLeftValW  = 110;
        const int16_t kDiagRightLblX = 242;
        const int16_t kDiagRightValX = 350;
        const int16_t kDiagRightLblW = 100;
        const int16_t kDiagRightValW = 120;

        for (int i = 0; i < kDiagRows; ++i)
        {
            const bool isLeft = (i < kDiagLeftRows);
            const int  row    = isLeft ? i : (i - kDiagLeftRows);
            const int16_t y   = kDiagTopY + (int16_t)(row * kDiagLineH);

            const int16_t lblX = isLeft ? kDiagLeftLblX  : kDiagRightLblX;
            const int16_t valX = isLeft ? kDiagLeftValX  : kDiagRightValX;
            const int16_t lblW = isLeft ? kDiagLeftLblW  : kDiagRightLblW;
            const int16_t valW = isLeft ? kDiagLeftValW  : kDiagRightValW;

            diag_label_[i].setPosition(lblX, y, lblW, kDiagLineH);
            diag_label_[i].setColor(kColText);
            diag_label_[i].setTypedText(TypedText(T_DYN_S));
            utf8ToBuf("", diag_label_buf_[i],
                      sizeof(diag_label_buf_[i]) / sizeof(diag_label_buf_[i][0]));
            diag_label_[i].setWildcard(diag_label_buf_[i]);
            diag_label_[i].setVisible(false);
            add(diag_label_[i]);

            diag_value_[i].setPosition(valX, y, valW, kDiagLineH);
            diag_value_[i].setColor(kColTextDim);
            diag_value_[i].setTypedText(TypedText(T_DYN_S));
            utf8ToBuf("", diag_value_buf_[i],
                      sizeof(diag_value_buf_[i]) / sizeof(diag_value_buf_[i][0]));
            diag_value_[i].setWildcard(diag_value_buf_[i]);
            diag_value_[i].setVisible(false);
            add(diag_value_[i]);
        }
    }
}

void screenView::handleTickEvent()
{
    screenViewBase::handleTickEvent();

    /* Input is owned by Model::tick (runs before this on every frame). We
     * deliberately do NOT poll the rotary here - draining the same latch
     * from two places causes presses to be silently lost when both ticks
     * race for them. */

    /* Live pressure trace updates every tick on the THERAPY page. */
    if (cpap::getMenu().page() == cpap::Page::THERAPY)
    {
        updateTherapyPressureGraph();
    }

    /* Status bar refreshes once per second-ish to show live telemetry. */
    static uint32_t s_status_tick = 0U;
    if (++s_status_tick >= 60U)   /* TouchGFX tick ~ 60 Hz                    */
    {
        s_status_tick = 0U;
        cycleStatusIconDemo();    /* ICON DEMO - remove when icons are live */
        updateStatusBar();
        /* The Diagnostics page shows live sensor values, so it needs to
         * repaint at the same cadence as the status bar even when the
         * menu controller didn't flag a redraw. The Therapy page is
         * the same story (run timer + leak readout). */
        if (cpap::getMenu().page() == cpap::Page::DIAGNOSTICS)
        {
            renderDiagnosticsPage();
        }
        else if (cpap::getMenu().page() == cpap::Page::THERAPY)
        {
            renderTherapyPage();
        }
        else if (cpap::getMenu().page() == cpap::Page::WELCOME)
        {
            /* #87: the reminder appears once the card has been read. */
            renderWelcomePage();
            welcome_sub_text_.invalidate();
        }
        else if (cpap::getMenu().page() == cpap::Page::SUPPLIES)
        {
            /* #87: only the state column changes on its own (over hours),
             * so redraw a value only when its text changed. Re-rendering
             * and invalidating the whole page every second flickered. */
            static char s_last[kMaxRows][24];
            cpap::MenuController& m = cpap::getMenu();
            for (uint8_t r = 0; r < row_visible_count_; ++r)
            {
                const char* val = m.itemValue((uint8_t)(list_scroll_offset_ + r));
                if ((val != nullptr) && (strncmp(val, s_last[r], sizeof(s_last[r]) - 1U) != 0))
                {
                    (void)strncpy(s_last[r], val, sizeof(s_last[r]) - 1U);
                    s_last[r][sizeof(s_last[r]) - 1U] = '\0';
                    utf8ToBuf(val, row_value_buf_[r],
                              sizeof(row_value_buf_[r]) / sizeof(row_value_buf_[r][0]));
                    row_value_[r].invalidate();
                }
            }
        }
        else if (cpap::getMenu().page() == cpap::Page::LEARN_TUBE)
        {
            /* #85: step / result of the learn-circuit run. */
            renderInfoPage();
            welcome_big_text_.invalidate();
            welcome_sub_text_.invalidate();
        }
    }

    if (cpap::getMenu().needsRedraw())
    {
        DBG_D("UI", "render page=%u mode=%u sel=%u",
              (unsigned)cpap::getMenu().page(),
              (unsigned)cpap::getMenu().mode(),
              (unsigned)cpap::getMenu().selectedIndex());
        renderCurrentPage();
        cpap::getMenu().clearRedraw();
    }
}

void screenView::updateStatusBar()
{
    using namespace cpap;
    const Settings&     s   = getSettings();
    const SystemStatus& sys = getSystemStatus();

    /* Left = page name. */
    utf8ToBuf(getMenu().editorTitle(), status_left_buf_,
              sizeof(status_left_buf_) / sizeof(status_left_buf_[0]));
    status_left_text_.invalidate();

    /* Right side: 5 icon slots. Each one is hidden unless a condition
     * holds. The visibility setters below all run cheaply (no allocation,
     * no font work); only the icons whose state actually changes will
     * cause TouchGFX to repaint, because IconWidget::setIcon/setColor
     * already compare-before-invalidate when nothing changed. */

    /* Slot 0: Home icon - shown only on the HOME page so it doesn't
     * compete with the page-name text on every other screen. */
    icon_home_.setVisibleAndInvalidate(getMenu().page() == Page::HOME);

    /* Slot 1: Airplane mode toggle from Settings. */
    icon_airplane_.setVisibleAndInvalidate(s.airplane_mode == OnOff::ON);

    /* Slot 2: Cellular - if airplane mode is ON, the modem is off so we
     * suppress the cellular icon entirely (matches phone-OS behaviour).
     * Otherwise we pick the bar tower for the current signal level, or
     * the no-signal X if there is no service. */
    if (s.airplane_mode == OnOff::ON)
    {
        icon_cellular_.setVisibleAndInvalidate(false);
    }
    else
    {
        const uint16_t* mask = nullptr;
        colortype       col  = kColText;
        switch (sys.cellular)
        {
        case CellularSignal::NONE:
            mask = kIconNoCellular;
            col  = kColTextDim;            /* dim - we have no service     */
            break;
        case CellularSignal::BARS_1: mask = kIconCellularBars1; break;
        case CellularSignal::BARS_2: mask = kIconCellularBars2; break;
        case CellularSignal::BARS_3: mask = kIconCellularBars3; break;
        case CellularSignal::BARS_4: mask = kIconCellularBars4; break;
        }
        if (mask != nullptr)
        {
            icon_cellular_.setIcon(mask);
            icon_cellular_.setColor(col);
            icon_cellular_.setVisibleAndInvalidate(true);
        }
        else
        {
            icon_cellular_.setVisibleAndInvalidate(false);
        }
    }

    /* Slot 3: Bluetooth pairing status. Suppressed in airplane mode for
     * the same reason as cellular. */
    icon_bluetooth_.setVisibleAndInvalidate(
        (s.airplane_mode != OnOff::ON) && sys.bluetooth_connected);

    /* Slot 4: Humidifier state - mutually exclusive: NORMAL hides the
     * icon; FAULT/WARMING/COOLING each have their own glyph + colour. */
    switch (sys.humidifier)
    {
    case HumidifierStatus::NORMAL:
        icon_humidifier_.setVisibleAndInvalidate(false);
        break;
    case HumidifierStatus::FAULT:
        icon_humidifier_.setIcon(kIconHumFault);
        icon_humidifier_.setColor(kColIconWarn);
        icon_humidifier_.setVisibleAndInvalidate(true);
        break;
    case HumidifierStatus::WARMING:
        icon_humidifier_.setIcon(kIconHumWarming);
        icon_humidifier_.setColor(kColIconWarm);
        icon_humidifier_.setVisibleAndInvalidate(true);
        break;
    case HumidifierStatus::COOLING:
        icon_humidifier_.setIcon(kIconHumCooling);
        icon_humidifier_.setColor(kColIconCool);
        icon_humidifier_.setVisibleAndInvalidate(true);
        break;
    }
}

/* ---------- ICON DEMO -----------------------------------------------------
 *
 *  Temporary demo cycle so all status-bar icons can be visually verified
 *  before real telemetry drivers are wired up. Called once per second from
 *  handleTickEvent(). Each "stage" lasts ~2 s; one full lap is ~20 s.
 *
 *  Stages:
 *      0 : baseline   cell=NONE   bt=off   hum=NORMAL    -> X only
 *      1 : 1-bar      cell=BARS_1                        -> 1-bar tower
 *      2 : 2-bar      cell=BARS_2                        -> 2-bar tower
 *      3 : 3-bar      cell=BARS_3                        -> 3-bar tower
 *      4 : 4-bar      cell=BARS_4                        -> 4-bar tower
 *      5 : + BT       bt=on                              -> + Bluetooth
 *      6 : + warm     hum=WARMING                        -> + amber droplet '+'
 *      7 : cool       hum=COOLING                        -> blue droplet '-'
 *      8 : fault      hum=FAULT                          -> red triangle
 *      9 : reset      everything back to defaults
 *
 *  The Home icon is page-driven, not telemetry-driven, so it always shows
 *  on the HOME page regardless of the demo state. Airplane mode is owned
 *  by the user (Settings -> MORE -> Airplane Mode), so this demo doesn't
 *  touch it - flip it manually when you want to see that glyph.
 *
 *  To remove: delete this function, its declaration in screenView.hpp,
 *  and the caller line tagged "ICON DEMO" in handleTickEvent().
 * ------------------------------------------------------------------------- */
void screenView::cycleStatusIconDemo()
{
    using namespace cpap;

    /* This runs every ~1 s. Advance one stage every 2 ticks (~2 s). */
    static uint8_t s_tick  = 0;
    static uint8_t s_stage = 0;

    if (++s_tick < 2U)
    {
        return;
    }
    s_tick = 0U;

    enum : uint8_t {
        STAGE_BASELINE = 0,
        STAGE_CELL_1,
        STAGE_CELL_2,
        STAGE_CELL_3,
        STAGE_CELL_4,
        STAGE_PLUS_BT,
        STAGE_PLUS_HUM_WARM,
        STAGE_HUM_COOL,
        STAGE_HUM_FAULT,
        STAGE_RESET,
        STAGE_COUNT
    };

    SystemStatus& sys = getSystemStatus();

    switch (s_stage)
    {
    case STAGE_BASELINE:
        sys.cellular            = CellularSignal::NONE;
        sys.bluetooth_connected = false;
        sys.humidifier          = HumidifierStatus::NORMAL;
        break;
    case STAGE_CELL_1: sys.cellular = CellularSignal::BARS_1; break;
    case STAGE_CELL_2: sys.cellular = CellularSignal::BARS_2; break;
    case STAGE_CELL_3: sys.cellular = CellularSignal::BARS_3; break;
    case STAGE_CELL_4: sys.cellular = CellularSignal::BARS_4; break;
    case STAGE_PLUS_BT:
        sys.bluetooth_connected = true;
        break;
    case STAGE_PLUS_HUM_WARM:
        sys.humidifier = HumidifierStatus::WARMING;
        break;
    case STAGE_HUM_COOL:
        sys.humidifier = HumidifierStatus::COOLING;
        break;
    case STAGE_HUM_FAULT:
        sys.humidifier = HumidifierStatus::FAULT;
        break;
    case STAGE_RESET:
        sys.cellular            = CellularSignal::NONE;
        sys.bluetooth_connected = false;
        sys.humidifier          = HumidifierStatus::NORMAL;
        break;
    default: break;
    }

    s_stage = (s_stage + 1U) % STAGE_COUNT;
}

void screenView::hideAllPageWidgets()
{
    for (int i = 0; i < kMaxRows; ++i)
    {
        row_bg_[i].setVisible(false);
        row_label_[i].setVisible(false);
        row_value_[i].setVisible(false);
    }
    editor_value_text_.setVisible(false);
    editor_hint_text_.setVisible(false);
    welcome_big_text_.setVisible(false);
    welcome_sub_text_.setVisible(false);
    therapy_pressure_text_.setVisible(false);
    therapy_live_big_text_.setVisible(false);
    therapy_summary_text_.setVisible(false);
    therapy_setpoint_line_.setVisible(false);
    for (int i = 0; i < kWaveBars; ++i)
    {
        wave_bars_[i].setVisible(false);
    }
    for (int i = 0; i < kDiagRows; ++i)
    {
        diag_label_[i].setVisible(false);
        diag_value_[i].setVisible(false);
    }
}

void screenView::renderCurrentPage()
{
    using namespace cpap;
    MenuController& m = getMenu();

    hideAllPageWidgets();
    updateStatusBar();

    /* Title text always reflects the current page. */
    utf8ToBuf(m.editorTitle(), title_buf_,
              sizeof(title_buf_) / sizeof(title_buf_[0]));
    title_text_.invalidate();

    switch (m.page())
    {
    case Page::WELCOME:
        renderWelcomePage();
        break;
    case Page::HOME:
    case Page::MY_OPTIONS:
    case Page::MORE:
    case Page::SUPPLIES:
        renderListPage();
        break;
    case Page::VALUE_THERAPY_MODE:
    case Page::VALUE_CPAP_PRESSURE:
    case Page::VALUE_AUTOSET_MIN:
    case Page::VALUE_AUTOSET_MAX:
    case Page::VALUE_RAMP:
    case Page::VALUE_RAMP_START:
    case Page::VALUE_EPR:
    case Page::VALUE_EPR_TYPE:
    case Page::VALUE_EPR_LEVEL:
    case Page::VALUE_SMART_START:
    case Page::VALUE_SMART_STOP:
    case Page::VALUE_TUBE_TEMP_MODE:
    case Page::VALUE_TUBE_TEMP_C:
    case Page::VALUE_CLIMATE:
    case Page::VALUE_HUMIDITY:
    case Page::VALUE_AIRPLANE_MODE:
        renderValueEditorPage();
        break;
    case Page::MY_SLEEP_VIEW:
    case Page::MASK_FIT:
    case Page::LEARN_TUBE:
    case Page::SUPPLY_CONFIRM:
    case Page::MYAIR:
        renderInfoPage();
        break;
    case Page::THERAPY:
        resetTherapyPressureHistory(Therapy_GetMeasuredCmh2o());
        renderTherapyPage();
        break;
    case Page::DIAGNOSTICS:
        renderDiagnosticsPage();
        break;
    default:
        break;
    }

    /* Force the entire content area to repaint so the previous page's pixels
     * are cleared. The Box widgets do the actual clearing; we just invalidate
     * the region. */
    Rect content(0, kContentY, kScreenW, kContentH);
    invalidateRect(content);
}

void screenView::renderListPage()
{
    using namespace cpap;
    MenuController& m = getMenu();
    const uint8_t count = m.itemCount();
    const uint8_t sel   = m.selectedIndex();

    /* Keep the selected item inside the visible window. Note we work in
     * uint8_t throughout - all our list lengths and indices are small. */
    if (sel < list_scroll_offset_)
    {
        list_scroll_offset_ = sel;
    }
    else if (sel >= (uint8_t)(list_scroll_offset_ + kMaxRows))
    {
        list_scroll_offset_ = (uint8_t)(sel - kMaxRows + 1);
    }
    /* Don't leave empty slots at the bottom when the cursor walks back up
     * after a previous scroll. */
    if ((uint16_t)list_scroll_offset_ + kMaxRows > count)
    {
        list_scroll_offset_ = (count > kMaxRows) ? (uint8_t)(count - kMaxRows) : 0U;
    }

    uint8_t visible = (uint8_t)(count - list_scroll_offset_);
    if (visible > kMaxRows) visible = kMaxRows;
    row_visible_count_ = visible;

    /* Hide any rows beyond the visible window left over from a longer list. */
    for (uint8_t r = visible; r < kMaxRows; ++r)
    {
        row_bg_   [r].setVisible(false);
        row_label_[r].setVisible(false);
        row_value_[r].setVisible(false);
    }

    for (uint8_t r = 0; r < visible; ++r)
    {
        const uint8_t i = (uint8_t)(list_scroll_offset_ + r);
        const bool isSel = (i == sel);
        row_bg_[r].setColor(isSel ? kColRowSel : kColContentBg);
        row_bg_[r].setVisible(true);

        utf8ToBuf(m.itemLabel(i), row_label_buf_[r],
                  sizeof(row_label_buf_[r]) / sizeof(row_label_buf_[r][0]));
        row_label_[r].setColor(isSel ? kColBg : kColText);
        row_label_[r].setVisible(true);

        const char* val = m.itemValue(i);
        if (val != nullptr)
        {
            utf8ToBuf(val, row_value_buf_[r],
                      sizeof(row_value_buf_[r]) / sizeof(row_value_buf_[r][0]));
            row_value_[r].setColor(isSel ? kColBg : kColTextDim);
            row_value_[r].setVisible(true);
        }
        else
        {
            row_value_[r].setVisible(false);
        }
    }
}

void screenView::renderValueEditorPage()
{
    using namespace cpap;
    MenuController& m = getMenu();

    utf8ToBuf(m.editorValueText(), editor_value_buf_,
              sizeof(editor_value_buf_) / sizeof(editor_value_buf_[0]));
    editor_value_text_.setVisible(true);

    utf8ToBuf(m.editorHint(), editor_hint_buf_,
              sizeof(editor_hint_buf_) / sizeof(editor_hint_buf_[0]));
    editor_hint_text_.setVisible(true);
}

void screenView::renderInfoPage()
{
    using namespace cpap;
    MenuController& m = getMenu();

    /* Repurpose the welcome widgets for the info text. */
    welcome_big_text_.setColor(kColText);
    welcome_sub_text_.setColor(kColTextDim);

    const char* big = "";
    switch (m.page())
    {
    case Page::MY_SLEEP_VIEW: big = "Sleep view (placeholder)"; break;
    case Page::MASK_FIT:      big = "Mask Fit check";           break;
    case Page::LEARN_TUBE:    big = m.learnTubeText();          break;
    case Page::SUPPLY_CONFIRM: big = m.supplyConfirmText();     break;
    case Page::MYAIR:         big = "myAir / Bluetooth";        break;
    default:                  big = "";                         break;
    }
    utf8ToBuf(big, welcome_big_buf_,
              sizeof(welcome_big_buf_) / sizeof(welcome_big_buf_[0]));
    welcome_big_text_.setVisible(true);

    utf8ToBuf(m.editorHint(), welcome_sub_buf_,
              sizeof(welcome_sub_buf_) / sizeof(welcome_sub_buf_[0]));
    welcome_sub_text_.setVisible(true);
}

void screenView::renderWelcomePage()
{
    utf8ToBuf("Welcome", welcome_big_buf_,
              sizeof(welcome_big_buf_) / sizeof(welcome_big_buf_[0]));
    welcome_big_text_.setColor(kColText);
    welcome_big_text_.setVisible(true);

    /* #87: name what is due; the full list is under MORE > Supplies. */
    const char* sub = "Press the knob to begin";
    char due[40];
    if (Service_AnyDue())
    {
        const char* items = Service_DueText();
        const int n = snprintf(due, sizeof(due), "Replace: %s", items);
        sub = ((n > 0) && (static_cast<size_t>(n) < sizeof(due)))
                  ? due : "Supplies due: see MORE > Supplies";
    }
    utf8ToBuf(sub, welcome_sub_buf_,
              sizeof(welcome_sub_buf_) / sizeof(welcome_sub_buf_[0]));
    welcome_sub_text_.setColor(Service_AnyDue() ? kColText : kColTextDim);
    welcome_sub_text_.setVisible(true);
}

void screenView::renderTherapyPage()
{
    /* Top line: state, current pressure / target pressure.
     * The state name comes straight out of the C therapy module so the
     * UI label and the back-end log line are guaranteed to agree. */
    char buf[64];
    const float meas   = Therapy_GetMeasuredCmh2o();
    const float target = Therapy_GetTargetPressure();
    const TherapyState_t st = Therapy_GetState();
    const TherapyMode_t mode = Therapy_GetMode();

    if (Therapy_IsMaskOffActive())
    {
        (void)snprintf(buf, sizeof(buf),
                       "MASK OFF: blower at low speed   %.1f / %.1f cmH2O",
                       static_cast<double>(meas),
                       static_cast<double>(target));
    }
    else if (st == THERAPY_STATE_PRECHECK)
    {
        /* #83: blower spin self-test, ~3 s before the ramp / therapy. */
        (void)snprintf(buf, sizeof(buf), "Self-test: checking blower...");
    }
    else if (st == THERAPY_STATE_FAULT)
    {
        (void)snprintf(buf, sizeof(buf),
                       "FAULT: %s",
                       Therapy_GetFaultName());
    }
    else if ((st == THERAPY_STATE_RAMP) && Therapy_GetRampAuto()
             && (Therapy_GetAutoRampPhase() == 0U))
    {
        /* #38: AutoRamp holds the start pressure until sleep onset. */
        (void)snprintf(buf, sizeof(buf),
                       "AutoRamp: %.1f cmH2O, waiting for sleep",
                       static_cast<double>(meas));
    }
    else if (st == THERAPY_STATE_RAMP)
    {
        const uint32_t r = Therapy_GetRampRemainingSeconds();
        if (mode == THERAPY_MODE_CPAP)
        {
            (void)snprintf(buf, sizeof(buf),
                           "Ramping: %.1f / %.1f cmH2O   %lu s left",
                           static_cast<double>(meas),
                           static_cast<double>(target),
                           static_cast<unsigned long>(r));
        }
        else
        {
            (void)snprintf(buf, sizeof(buf),
                           "%s Ramping: %.1f / %.1f cmH2O (%.1f-%.1f)  %lu s",
                           Therapy_GetModeName(),
                           static_cast<double>(meas),
                           static_cast<double>(target),
                           static_cast<double>(Therapy_GetApapMinPressure()),
                           static_cast<double>(Therapy_GetApapMaxPressure()),
                           static_cast<unsigned long>(r));
        }
    }
    else
    {
        if (mode == THERAPY_MODE_CPAP)
        {
            if (Therapy_IsEprEnabled())
            {
                (void)snprintf(buf, sizeof(buf),
                               "%s  EPR:%s: %.1f / %.1f cmH2O",
                               Therapy_GetStateName(),
                               Epr_GetPhaseName(),
                               static_cast<double>(meas),
                               static_cast<double>(target));
            }
            else
            {
                (void)snprintf(buf, sizeof(buf),
                               "%s: %.1f / %.1f cmH2O",
                               Therapy_GetStateName(),
                               static_cast<double>(meas),
                               static_cast<double>(target));
            }
        }
        else if (Therapy_IsEprEnabled())
        {
            (void)snprintf(buf, sizeof(buf),
                           "%s  EPR:%s: %.1f / %.1f (%.1f-%.1f) cmH2O",
                           Therapy_GetModeName(),
                           Epr_GetPhaseName(),
                           static_cast<double>(meas),
                           static_cast<double>(target),
                           static_cast<double>(Therapy_GetApapMinPressure()),
                           static_cast<double>(Therapy_GetApapMaxPressure()));
        }
        else
        {
            (void)snprintf(buf, sizeof(buf),
                           "%s: %.1f / %.1f (%.1f-%.1f) cmH2O",
                           Therapy_GetModeName(),
                           static_cast<double>(meas),
                           static_cast<double>(target),
                           static_cast<double>(Therapy_GetApapMinPressure()),
                           static_cast<double>(Therapy_GetApapMaxPressure()));
        }
    }
    utf8ToBuf(buf, therapy_pressure_buf_,
              sizeof(therapy_pressure_buf_) / sizeof(therapy_pressure_buf_[0]));
    therapy_pressure_text_.setColor(
        (st == THERAPY_STATE_FAULT) ? kColIconWarn
        : (Therapy_IsMaskOffActive() ? kColIconWarn : kColText));
    therapy_pressure_text_.setVisible(true);

    /* Bottom line: run time + leak status. */
    const uint32_t total_s = Therapy_GetRunSeconds();
    const uint32_t hh      = total_s / 3600U;
    const uint32_t mm      = (total_s / 60U) % 60U;
    const uint32_t ss      = total_s % 60U;
    const float leak       = Therapy_GetLeakSlm();
    const char* mask_word  = Therapy_IsMaskOffActive() ? "MASK OFF"
                            : (Therapy_IsLeakLow()       ? "LOW FLOW"
                            : Therapy_GetLeakSeverityName());

    const char* apap_suffix = "";
    if (mode != THERAPY_MODE_CPAP && st == THERAPY_STATE_RUNNING)
    {
        const char* apap_name = Therapy_GetApapStateName();
        if (apap_name != nullptr && apap_name[0] != '\0')
        {
            apap_suffix = apap_name;
        }
    }

    if (apap_suffix[0] != '\0')
    {
        (void)snprintf(buf, sizeof(buf),
                       "Time: %lu:%02lu:%02lu   Leak: %.0f slm  (%s)  APAP:%s",
                       static_cast<unsigned long>(hh),
                       static_cast<unsigned long>(mm),
                       static_cast<unsigned long>(ss),
                       static_cast<double>(leak),
                       mask_word,
                       apap_suffix);
    }
    else
    {
        (void)snprintf(buf, sizeof(buf),
                       "Time: %lu:%02lu:%02lu   Leak: %.0f slm  (%s)%s",
                       static_cast<unsigned long>(hh),
                       static_cast<unsigned long>(mm),
                       static_cast<unsigned long>(ss),
                       static_cast<double>(leak),
                       mask_word,
                       Therapy_IsEprEnabled() ? "  EPR" : "");
    }
    utf8ToBuf(buf, therapy_summary_buf_,
              sizeof(therapy_summary_buf_) / sizeof(therapy_summary_buf_[0]));
    therapy_summary_text_.setVisible(true);
    therapy_summary_text_.setColor(
        (Therapy_IsMaskOffActive() || Therapy_IsLeakHigh() || Therapy_IsLeakLow())
            ? kColIconWarn : kColTextDim);

    therapy_setpoint_line_.setVisible(true);
    therapy_live_big_text_.setVisible(true);
    for (int i = 0; i < kWaveBars; ++i)
    {
        wave_bars_[i].setVisible(true);
    }
    updateTherapyPressureGraph();
}

/* Helper: pick a status word and a colour for a self-test cell. */
static void diagTestCell(DiagnosticsTestState st,
                         const char** out_text,
                         touchgfx::colortype* out_color)
{
    switch (st)
    {
    case DIAG_TEST_PASS:    *out_text = "PASS";    *out_color = kColTestPass; break;
    case DIAG_TEST_FAIL:    *out_text = "FAIL";    *out_color = kColIconWarn; break;
    case DIAG_TEST_RUNNING: *out_text = "RUN...";  *out_color = kColIconWarm; break;
    case DIAG_TEST_NO_CARD: *out_text = "NO CARD"; *out_color = kColIconWarm; break;
    case DIAG_TEST_NOT_RUN:
    default:                *out_text = "---";     *out_color = kColTextDim;  break;
    }
}

void screenView::renderDiagnosticsPage()
{
    /* Pull a single consistent snapshot. The struct is small (a few dozen
     * bytes) so the cost of a fresh copy each tick is negligible compared
     * to the cost of repainting the screen. */
    DiagnosticsSnapshot_t snap;
    Diagnostics_GetSnapshot(&snap);

    /* ---------- Left column: SELF-TESTS ---------- */
    static const char* kLeftLabels[kDiagLeftRows] = {
        "SELF-TESTS",
        "SD card",
        "NFC",
        "SDRAM",
        "QSPI flash"
    };

    /* Header row: just the title in the bright colour, no value. */
    utf8ToBuf(kLeftLabels[0], diag_label_buf_[0],
              sizeof(diag_label_buf_[0]) / sizeof(diag_label_buf_[0][0]));
    diag_label_[0].setColor(kColText);
    diag_label_[0].setVisible(true);
    utf8ToBuf("", diag_value_buf_[0],
              sizeof(diag_value_buf_[0]) / sizeof(diag_value_buf_[0][0]));
    diag_value_[0].setVisible(false);

    const DiagnosticsTestState testStates[4] = {
        snap.sd_state, snap.nfc_state, snap.sdram_state, snap.qspi_state
    };
    for (int i = 1; i < kDiagLeftRows; ++i)
    {
        utf8ToBuf(kLeftLabels[i], diag_label_buf_[i],
                  sizeof(diag_label_buf_[i]) / sizeof(diag_label_buf_[i][0]));
        diag_label_[i].setColor(kColTextDim);
        diag_label_[i].setVisible(true);

        const char*         word = "---";
        touchgfx::colortype col  = kColTextDim;
        diagTestCell(testStates[i - 1], &word, &col);
        utf8ToBuf(word, diag_value_buf_[i],
                  sizeof(diag_value_buf_[i]) / sizeof(diag_value_buf_[i][0]));
        diag_value_[i].setColor(col);
        diag_value_[i].setVisible(true);
    }

    /* ---------- Right column: LIVE SENSORS ---------- */
    /* Local scratch buffer reused across all the rows so we don't burn
     * stack on per-row arrays. snprintf truncates safely on overflow. */
    char tmp[24];

    /* Right header. */
    const int hdr = kDiagLeftRows;
    utf8ToBuf("LIVE SENSORS", diag_label_buf_[hdr],
              sizeof(diag_label_buf_[hdr]) / sizeof(diag_label_buf_[hdr][0]));
    diag_label_[hdr].setColor(kColText);
    diag_label_[hdr].setVisible(true);
    utf8ToBuf("", diag_value_buf_[hdr],
              sizeof(diag_value_buf_[hdr]) / sizeof(diag_value_buf_[hdr][0]));
    diag_value_[hdr].setVisible(false);

    /* For each row: set label, format value into tmp, push to the buffer,
     * pick a colour. The valid flags from the snapshot let us dim a value
     * that hasn't been refreshed by its driver (e.g. CRC errors on the
     * Sensirion bus) so the user can tell stale from live. */
    struct LiveRow {
        const char* label;
        float       value;
        const char* unit;
        bool        valid;
        int         decimals;
    };
    const LiveRow rows[7] = {
        { "Humidity",  snap.humidity_pct,        " %",   snap.humidity_valid != 0U,  1 },
        { "Hum temp",  snap.humidifier_temp_c,   " C",   snap.humidity_valid != 0U,  1 },
        { "Tube temp", snap.tube_temp_c,         " C",   snap.tube_temp_valid != 0U, 1 },
        { "P-Flow",    snap.pressure_flow_mbar,  " mbar", true, 2 },
        { "P-Atmos",   snap.pressure_atmos_mbar, " mbar", true, 1 },
        { "P-Mask",    snap.pressure_mask_mbar,  " mbar", true, 2 },
        { "P-Analog",  snap.pressure_analog_mbar," mbar", true, 2 },
    };

    for (int i = 0; i < kDiagRightRows - 1; ++i)
    {
        const int idx = hdr + 1 + i;
        utf8ToBuf(rows[i].label, diag_label_buf_[idx],
                  sizeof(diag_label_buf_[idx]) / sizeof(diag_label_buf_[idx][0]));
        diag_label_[idx].setColor(kColTextDim);
        diag_label_[idx].setVisible(true);

        if (rows[i].decimals == 1)
        {
            (void)snprintf(tmp, sizeof(tmp), "%.1f%s",
                           (double)rows[i].value, rows[i].unit);
        }
        else
        {
            (void)snprintf(tmp, sizeof(tmp), "%.2f%s",
                           (double)rows[i].value, rows[i].unit);
        }
        utf8ToBuf(tmp, diag_value_buf_[idx],
                  sizeof(diag_value_buf_[idx]) / sizeof(diag_value_buf_[idx][0]));
        diag_value_[idx].setColor(rows[i].valid ? kColText : kColTextDim);
        diag_value_[idx].setVisible(true);
    }

    /* Force a content-area repaint so the previous page's pixels clear. */
    for (int i = 0; i < kDiagRows; ++i)
    {
        diag_label_[i].invalidate();
        diag_value_[i].invalidate();
    }
}

void screenView::resetTherapyPressureHistory(float seed_cmh2o)
{
    therapy_pressure_hist_[0] = seed_cmh2o;
    therapy_hist_count_       = 1U;
    therapy_hist_decim_       = 0U;
    for (int i = 1; i < kWaveBars; ++i)
    {
        therapy_pressure_hist_[i] = seed_cmh2o;
    }
}

void screenView::updateTherapyPressureGraph()
{
    const float meas    = Therapy_GetMeasuredCmh2o();
    const float target  = Therapy_GetTargetPressure();
    const float p_range = kTherapyGraphPmax - kTherapyGraphPmin;

    /* ~10 Hz history at 60 Hz TouchGFX tick -> ~4 s visible window. */
    if (therapy_hist_count_ > 0U)
    {
        if (therapy_hist_count_ < static_cast<uint8_t>(kWaveBars))
        {
            therapy_pressure_hist_[therapy_hist_count_ - 1U] = meas;
        }
        else
        {
            therapy_pressure_hist_[kWaveBars - 1] = meas;
        }
    }

    if (++therapy_hist_decim_ >= 6U)
    {
        therapy_hist_decim_ = 0U;
        if (therapy_hist_count_ < static_cast<uint8_t>(kWaveBars))
        {
            therapy_pressure_hist_[therapy_hist_count_] = meas;
            therapy_hist_count_++;
        }
        else
        {
            for (int i = 0; i < kWaveBars - 1; ++i)
            {
                therapy_pressure_hist_[i] = therapy_pressure_hist_[i + 1];
            }
            therapy_pressure_hist_[kWaveBars - 1] = meas;
        }
    }

    const int16_t bars_y    = kContentY + 30;
    const int16_t bars_h    = kTherapyGraphH;
    const int16_t bars_left = 20;
    const int16_t bars_w    = kScreenW - 2 * bars_left;
    const int16_t bar_w     = bars_w / kWaveBars;
    const int16_t graph_bot = bars_y + bars_h;

    auto pressureToY = [&](float p) -> int16_t
    {
        float frac = (p - kTherapyGraphPmin) / p_range;
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        return (int16_t)(graph_bot - (int16_t)(frac * (float)bars_h));
    };

    /* Target pressure reference line (amber). */
    {
        int16_t y_sp = pressureToY(target);
        if (y_sp < bars_y) y_sp = bars_y;
        if (y_sp > graph_bot - 2) y_sp = graph_bot - 2;
        Rect r = therapy_setpoint_line_.getRect();
        therapy_setpoint_line_.moveTo(r.x, y_sp);
        therapy_setpoint_line_.invalidate();
    }

    /* Bar trace: left = older, right = newer. */
    for (int i = 0; i < kWaveBars; ++i)
    {
        float p = kTherapyGraphPmin;
        if (therapy_hist_count_ == 0U)
        {
            p = meas;
        }
        else if (i < static_cast<int>(therapy_hist_count_))
        {
            p = therapy_pressure_hist_[i];
        }

        int16_t y_top = pressureToY(p);
        int16_t hh = graph_bot - y_top;
        if (hh < 2) hh = 2;
        if (hh > bars_h) hh = bars_h;

        int16_t x = bars_left + (int16_t)(i * bar_w);
        wave_bars_[i].moveTo(x, y_top);
        wave_bars_[i].setHeight(hh);
        wave_bars_[i].invalidate();
    }

    char buf[16];
    (void)snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(meas));
    utf8ToBuf(buf, therapy_live_big_buf_,
              sizeof(therapy_live_big_buf_) / sizeof(therapy_live_big_buf_[0]));
    therapy_live_big_text_.invalidate();
}
