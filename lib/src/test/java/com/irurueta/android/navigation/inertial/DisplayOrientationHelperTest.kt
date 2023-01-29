package com.irurueta.android.navigation.inertial

import android.content.Context
import android.os.Build
import android.view.Display
import android.view.Surface
import android.view.WindowManager
import androidx.test.core.app.ApplicationProvider
import io.mockk.clearAllMocks
import io.mockk.every
import io.mockk.mockk
import io.mockk.spyk
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@Suppress("DEPRECATION")
@RunWith(RobolectricTestRunner::class)
class DisplayOrientationHelperTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAndNoDisplay_returns0() {
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(null)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAndZeroRotation_returns0() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd90Rotation_returns90() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_90)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        assertEquals(90.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd180Rotation_returns180() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_180)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        assertEquals(180.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd270Rotation_returns180() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_270)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        assertEquals(270.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAndZeroRotation_returns0() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val windowManager = mockk<WindowManager>()
        every { windowManager.defaultDisplay }.returns(display)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd90Rotation_returns90() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_90)
        val windowManager = mockk<WindowManager>()
        every { windowManager.defaultDisplay }.returns(display)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(90.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd180Rotation_returns180() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_180)
        val windowManager = mockk<WindowManager>()
        every { windowManager.defaultDisplay }.returns(display)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(180.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd270Rotation_returns270() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_270)
        val windowManager = mockk<WindowManager>()
        every { windowManager.defaultDisplay }.returns(display)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(270.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAndNoDisplay_returns0() {
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(null)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAndZeroRotation_returns0() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd90Rotation_returnsPiDividedBy2() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_90)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        assertEquals(
            Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd180Rotation_returnsPi() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_180)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        assertEquals(Math.PI, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd270Rotation_returns3PiDividedBy2() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_270)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        assertEquals(
            3.0 * Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAndZeroRotation_returns0() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val windowManager = mockk<WindowManager>()
        every { windowManager.defaultDisplay }.returns(display)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd90Rotation_returnsPiDividedBy2() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_90)
        val windowManager = mockk<WindowManager>()
        every { windowManager.defaultDisplay }.returns(display)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(
            Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd180Rotation_returnsPi() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_180)
        val windowManager = mockk<WindowManager>()
        every { windowManager.defaultDisplay }.returns(display)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(Math.PI, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd270Rotation_returns3PiDividedBy2() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_270)
        val windowManager = mockk<WindowManager>()
        every { windowManager.defaultDisplay }.returns(display)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(
            3.0 * Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }
}