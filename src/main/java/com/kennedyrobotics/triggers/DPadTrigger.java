package com.kennedyrobotics.triggers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DPadTrigger extends Trigger {

  public enum DPad {
    kUp, 
    KDown,
    KLeft,
    kRight
  }

  public DPadTrigger(XboxController controller, DPad dpad) {
    super(() -> {

      int pov = controller.getPOV();

      DPad current;

      switch (pov) {
        case 0:
          current = DPad.kUp;
          break;
        case 90:
          current = DPad.kRight;
          break;
        case 180:
          current = DPad.KDown;
          break;
        case 270:
          current = DPad.KLeft;
          break;
        default:
          return false;
      }

      return dpad == current;
    });
  }
}