vim.keymap.set("n", "<C-b>", "<cmd>make<CR>");
vim.keymap.set("n", "<C-p>", function () vim.fn.jobstart("run.bat") end);
