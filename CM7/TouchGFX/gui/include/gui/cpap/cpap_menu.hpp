/**
 * cpap_menu.hpp - Page hierarchy + rotary-driven navigation state machine.
 *
 * The screen view owns one MenuController. The view feeds in rotary detents
 * and button presses via the input methods, then asks the controller for the
 * page id, selected index, and per-page item list to redraw.
 *
 * Two interaction modes:
 *   - LIST     : rotating moves the selection cursor; click enters the item.
 *   - VALUE_EDIT: rotating changes the value of the focused setting;
 *                 click commits and returns to the parent list page.
 *
 * The view holds the rendering responsibility; this class is purely logic.
 */
#ifndef CPAP_MENU_HPP
#define CPAP_MENU_HPP

#include <stdint.h>

namespace cpap
{

enum class Page : uint8_t
{
    WELCOME = 0,
    HOME,                /* list: MY OPTIONS, MY SLEEP VIEW, MORE             */
    MY_OPTIONS,          /* list: Mode / CPAP or Autoset pressures / Ramp  */
    VALUE_THERAPY_MODE,  /* CPAP / Autoset / Autoset for her               */
    VALUE_CPAP_PRESSURE, /* fixed CPAP therapy pressure (4.0 - 25.0 cmH2O) */
    VALUE_AUTOSET_MIN,   /* Autoset range minimum                          */
    VALUE_AUTOSET_MAX,   /* Autoset range maximum                          */
    VALUE_RAMP,          /* value editor                                      */
    VALUE_RAMP_START,    /* ramp start pressure (#5)                          */
    VALUE_EPR,
    VALUE_EPR_TYPE,
    VALUE_EPR_LEVEL,
    VALUE_SMART_START,
    VALUE_SMART_STOP,
    VALUE_TUBE_TEMP_MODE,
    VALUE_TUBE_TEMP_C,   /* shown only when tube_temp_mode == MANUAL          */
    VALUE_CLIMATE,
    VALUE_HUMIDITY,
    MY_SLEEP_VIEW,       /* informational page                                */
    MORE,                /* list: Mask Fit, Airplane Mode, myAir, Diagnostics */
    MASK_FIT,            /* informational + cancel                            */
    VALUE_AIRPLANE_MODE,
    MYAIR,               /* informational                                     */
    DIAGNOSTICS,         /* live peripheral test results + sensor readings    */
    THERAPY,             /* dynamic pulse + summary                           */
    LEARN_TUBE,          /* #85: learn circuit, tube end open                 */
    SUPPLIES,            /* #87: list of consumables and their state          */
    SUPPLY_CONFIRM,      /* #87: "replaced?" Yes / No                         */
    NUM_PAGES
};

enum class Mode : uint8_t
{
    LIST,        /* rotary -> selection, click -> enter                       */
    VALUE_EDIT,  /* rotary -> change value, click -> commit + back            */
    INFO         /* rotary ignored, click -> back to parent                   */
};

class MenuController
{
public:
    MenuController();

    /* Input from the rotary driver. */
    void onRotaryDelta(int delta);    /* one detent = +/-1                    */
    void onButtonPress();             /* single click                         */
    void onButtonLongPress();         /* reserved: future use (e.g. global    */
                                      /*           back-to-home)              */

    /* State queries used by the view. */
    Page    page()           const { return current_page_; }
    Mode    mode()           const { return current_mode_; }
    uint8_t selectedIndex()  const { return selected_index_; }
    uint8_t itemCount()      const;  /* number of items on the current page  */
    bool    therapyRunning() const { return therapy_running_; }

    /* True if the view should redraw because state changed since the last
     * call to redraw(). The flag is sticky until the view consumes it. */
    bool    needsRedraw()    const { return needs_redraw_; }
    void    clearRedraw()          { needs_redraw_ = false; }
    void    requestRedraw()        { needs_redraw_ = true; }

    /* Returns the printable label for the given list item on the current
     * list page. nullptr if out of range or the current page is not a list. */
    const char* itemLabel(uint8_t idx) const;

    /* For list rows that display a current setting value on the right. */
    const char* itemValue(uint8_t idx) const;

    /* LEARN_TUBE page (#85): the centre line. editorHint() has the rest. */
    const char* learnTubeText() const;
    /* SUPPLY_CONFIRM page (#87): the item being confirmed. */
    const char* supplyConfirmText() const;

    /* For the value-editor pages, the big text shown in the centre. */
    const char* editorValueText() const;
    const char* editorTitle()     const;
    const char* editorHint()      const;   /* small text below the value     */

    /* For the THERAPY page. Driven by the back-end later; for now used for
     * a pulsing waveform in the view. */
    uint32_t therapyTickPhase() const { return therapy_phase_; }
    void     advanceTherapyPhase()    { therapy_phase_++; }

    /* Public so external code (e.g. a TouchGFX overlay or test harness) can
     * jump to a specific page. */
    void goTo(Page p, Mode m = Mode::LIST, uint8_t initial_index = 0);

private:
    Page    current_page_;
    Mode    current_mode_;
    uint8_t selected_index_;
    bool    needs_redraw_;

    /* THERAPY page state. */
    bool     therapy_running_;
    uint32_t therapy_phase_;

    /* Page-specific helpers. */
    void enterSelectedItem();
    void goBack();                    /* mode-aware: from VALUE_EDIT returns */
                                      /* to the parent list                  */
};

/* Singleton accessor: the view and any background poller share one. */
MenuController& getMenu();

} // namespace cpap

#endif // CPAP_MENU_HPP
