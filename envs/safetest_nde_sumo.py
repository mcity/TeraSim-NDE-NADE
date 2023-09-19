from .safetest_nde import SafeTestNDE

class SafeTestNDESUMO(SafeTestNDE):
    
    def on_step(self, ctx):
        # Make decisions and execute commands
        return self.should_continue_simulation()