import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("checklist", Path(__file__).with_name("update_checklist.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ChecklistTest(unittest.TestCase):
    def test_inventory_and_invalidation(self):
        with tempfile.TemporaryDirectory(prefix="suit-docs-") as tmp:
            root = Path(tmp)
            subprocess.run(["git", "init", "-q", str(root)], check=True)
            (root/".gitignore").write_text("build/\n")
            (root/"source.c").write_text("original\n")
            (root/"removed.c").write_text("removed\n")
            subprocess.run(["git", "add", "source.c", "removed.c"], cwd=root, check=True)
            (root/"removed.c").unlink()
            (root/"new.h").write_text("new\n")
            (root/"description.md").write_text("English description\n")
            (root/"build").mkdir()
            (root/"build/generated.c").write_text("generated\n")
            names=module.inventory(root)
            self.assertIn("source.c",names)
            self.assertIn("new.h",names)
            self.assertNotIn("removed.c",names)
            self.assertNotIn("build/generated.c",names)
            receipts={"source.c":{"doc":"description.md","sha256":module.digest(root/"source.c")}}
            text=module.render(root,names,receipts)
            self.assertIn("- [x] [source.c]",text)
            self.assertIn("- [ ] [new.h]",text)
            self.assertEqual(text,module.render(root,names,receipts))
            (root/"source.c").write_text("changed\n")
            self.assertIn("needs re-review",module.render(root,names,receipts))
            receipts["source.c"]["sha256"]=module.digest(root/"source.c")
            (root/"description.md").unlink()
            self.assertIn("- [ ] [source.c]",module.render(root,names,receipts))


if __name__ == "__main__":
    unittest.main()
