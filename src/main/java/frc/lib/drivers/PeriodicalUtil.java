package frc.lib.drivers;

import java.util.ArrayList;
import java.util.List;

public class PeriodicalUtil {
  private static List<Periodical> periodicls = new ArrayList<>();

  public static void registerPeriodic(Periodical periodic) {
    periodicls.add(periodic);
  }

  public static void runPeriodic() {
    periodicls.forEach((periodical) -> periodical.periodic());
  }
}

interface Periodical {
  public void periodic();
}
